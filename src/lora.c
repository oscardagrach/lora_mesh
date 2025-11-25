/**
 * @file lora.c
 * @brief LoRa radio interface for mesh network communication
 *
 * This module implements the LoRa radio interface for transmitting and
 * receiving packets in the mesh network. It handles radio configuration,
 * asynchronous transmission and reception, and packet queuing.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>

#include <led.h>
#include <lora.h>
#include <packet.h>
#include <ring_buffer.h>
#include <work_queue.h>
#include <config.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lora);

/**
 * @brief Default LoRa radio device from device tree
 */
#define DEFAULT_LORA_NODE DT_ALIAS(lora0)

/* Ensure the LoRa radio is defined in the device tree */
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_LORA_NODE, okay),
             "No default LoRa radio specified in device tree");

/**
 * @brief Stack size for the LoRa work queue
 */
#define STACK_SIZE 2048

/**
 * @brief Priority for the LoRa work queue (higher number = lower priority)
 */
#define WORK_QUEUE_PRIORITY 5

/**
 * @brief Priority for the TX signal handler thread
 */
#define TX_HANDLER_PRIORITY 7

/**
 * @brief Size of the TX signal handler thread stack
 */
#define TX_HANDLER_STACK_SIZE 1024

/**
 * @brief LED pulse duration in milliseconds for radio activity
 */
#define LED_PULSE_DURATION_MS RADIO_LED_PULSE_DURATION_MS

/* LoRa radio device */
const struct device *const lora_dev = DEVICE_DT_GET(DEFAULT_LORA_NODE);

/* Work item for packet transmission */
static struct k_work lora_tx_work;

/* Atomic counters for statistics */
atomic_t invalid_pkt = ATOMIC_INIT(0);
atomic_t drop_pkt = ATOMIC_INIT(0);

/* Dynamic node address derived from device ID */
uint32_t node_address = 0;

/* Signal for asynchronous TX completion */
static struct k_poll_signal lora_tx_signal;

/* Forward declaration of TX signal handler thread */
static void tx_signal_handler_thread(void *arg1, void *arg2, void *arg3);

/* Define the TX signal handler thread */
K_THREAD_DEFINE(tx_handler_tid, TX_HANDLER_STACK_SIZE,
                tx_signal_handler_thread, &lora_tx_signal, NULL, NULL,
                TX_HANDLER_PRIORITY, 0, 0);

/**
 * @brief LoRa receive callback function
 *
 * This function is called by the LoRa driver when a packet is received.
 * It validates the packet, adds RSSI and SNR information, and queues it
 * for processing.
 *
 * @param dev LoRa device that received the packet
 * @param data Pointer to the received data
 * @param size Size of the received data in bytes
 * @param rssi Received signal strength indicator in dBm
 * @param snr Signal-to-noise ratio in dB
 * @param user_data User data pointer (unused)
 */
static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t size,
                       int16_t rssi, int8_t snr, void *user_data)
{
    if (data == NULL) {
        LOG_ERR("NULL data received in LoRa callback");
        return;
    }

    hdr_t *hdr = (hdr_t *)data;

    /* Validate packet size */
    if (size < sizeof(hdr_t) || hdr->size != size) {
        LOG_WRN("Invalid packet size: expected %u, got %u", hdr->size, size);
        atomic_inc(&invalid_pkt);
        return;
    }

    /* Check if the packet is too large */
    if (size > MAX_PACKET_LEN) {
        LOG_WRN("Packet too large: %u bytes (max %u)", size, MAX_PACKET_LEN);
        atomic_inc(&invalid_pkt);
        return;
    }

    /* Ensure ring buffer has enough free space for incoming packet */
    size_t available_space = rb_space_get_rx();
    if (size > available_space) {
        LOG_ERR("RX buffer full: need %u bytes, have %u. Dropping packet.", 
                size, available_space);
        atomic_inc(&drop_pkt);
        return;
    }

    /* Indicate radio activity with LED */
    led_pulse_ms(LED_PULSE_DURATION_MS);

    /* Add signal information to packet header */
    hdr->rssi = rssi;
    hdr->snr = snr;

    /* Copy packet to ring buffer */
    int ret = rb_write_rx(data, size);

    /* Handle buffer overflow */
    if (ret < size) {
        LOG_ERR("RX buffer overflow: wrote %d of %u bytes", ret, size);
        return;
    }

    /* Submit work item to process the packet */
    k_work_submit_to_queue(&app_work_q, &packet_process_work);
    
    LOG_DBG("Received packet: %u bytes, RSSI: %d dBm, SNR: %d dB", 
            size, rssi, snr);
}

/**
 * @brief TX signal handler thread
 *
 * This thread waits for TX completion signals and handles post-transmission
 * tasks such as switching back to RX mode and checking for pending packets.
 *
 * @param arg1 Pointer to the TX signal
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void tx_signal_handler_thread(void *arg1, void *arg2, void *arg3)
{
    struct k_poll_signal *signal = (struct k_poll_signal *)arg1;
    struct k_poll_event event = K_POLL_EVENT_INITIALIZER(
        K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY,
        signal);

    while (1) {
        /* Wait for TX completion signal */
        int ret = k_poll(&event, 1, K_FOREVER);
        
        if (ret == 0 && signal->signaled) {
            LOG_DBG("TX completed (result: %d)", signal->result);
            
            /* Reconfigure radio for RX and re-enable RX callbacks */
            lora_radio_config(false);
            lora_recv_async(lora_dev, lora_rx_cb, NULL);

            /* Reset the signal for the next TX */
            k_poll_signal_reset(signal);

            /* Check if there's another packet pending in the TX ring buffer */
            size_t pending = rb_size_get_tx();
            if (pending > 0) {
                LOG_INF("Pending TX packets: %u bytes, resubmitting TX work", pending);
                k_work_submit_to_queue(&app_work_q, &lora_tx_work);
            }
        } else if (ret != 0) {
            LOG_ERR("k_poll error: %d", ret);
        }
    }
}

/**
 * @brief Send a packet from the TX ring buffer
 *
 * This function is called as a work item when packets are queued for
 * transmission. It reads a packet from the TX ring buffer and sends it
 * over the LoRa radio.
 *
 * @param work Pointer to the work item
 */
static void lora_tx_send(struct k_work *work)
{
    int ret = 0;
    uint8_t buf[MAX_PACKET_LEN] = {0};
    hdr_t *hdr = (hdr_t *)buf;

    /* First peek at the header to determine the full packet size */
    ret = rb_peek_tx(buf, sizeof(hdr_t));
    if (ret != sizeof(hdr_t)) {
        LOG_ERR("Failed to peek packet header: got %d bytes, expected %u", 
                ret, sizeof(hdr_t));
        return;
    }
    
    /* Validate packet size */
    if (hdr->size < sizeof(hdr_t) || hdr->size > MAX_PACKET_LEN) {
        LOG_ERR("Invalid packet size in TX buffer: %u", hdr->size);
        /* Discard the invalid packet */
        rb_read_tx(buf, sizeof(hdr_t));
        return;
    }
    
    /* Read the complete packet */
    ret = rb_read_tx(buf, hdr->size);
    if (ret != hdr->size) {
        LOG_ERR("Failed to read complete packet: got %d bytes, expected %u", 
                ret, hdr->size);
        return;
    }

    /* Indicate radio activity with LED */
    led_pulse_ms(LED_PULSE_DURATION_MS);

    /* Disable RX interrupts and set to TX mode */
    lora_recv_async(lora_dev, NULL, NULL);
    lora_radio_config(true);

    /* Reset the TX signal before initiating TX */
    k_poll_signal_reset(&lora_tx_signal);

    /* Set source address in packet header */
    hdr->src = NODE_ADDRESS;

    LOG_INF("Transmitting packet: %u bytes, type %u, dst 0x%08X", 
            hdr->size, hdr->type, hdr->dst);

    /* Initiate asynchronous TX */
    ret = lora_send_async(lora_dev, buf, hdr->size, &lora_tx_signal);
    if (ret != 0) {
        LOG_ERR("Asynchronous LoRa send failed: %d", ret);
        
        /* Reconfigure radio for RX in case of error */
        lora_radio_config(false);
        lora_recv_async(lora_dev, lora_rx_cb, NULL);
    }
}

/**
 * @brief Transmit a packet over LoRa
 *
 * Queues a packet for transmission over the LoRa radio.
 * The transmission is handled asynchronously by a work queue.
 *
 * @param buf Pointer to the packet buffer
 * @param size Size of the packet in bytes
 */
void lora_tx(uint8_t *buf, uint8_t size)
{
    if (buf == NULL) {
        LOG_ERR("NULL buffer provided to lora_tx");
        return;
    }

    if (size < sizeof(hdr_t) || size > MAX_PACKET_LEN) {
        LOG_ERR("Invalid packet size: %u", size);
        return;
    }

    /* Write packet to TX ring buffer */
    int ret = rb_write_tx(buf, size);
    if (ret != size) {
        LOG_ERR("Failed to write packet to TX buffer: wrote %d of %u bytes", 
                ret, size);
        return;
    }

    /* Submit work item to transmit the packet */
    k_work_submit_to_queue(&app_work_q, &lora_tx_work);
    LOG_DBG("Queued %u bytes for transmission", size);
}

/**
 * @brief Configure the LoRa radio for TX or RX
 *
 * Sets the LoRa radio parameters for either transmission or reception.
 *
 * @param tx True for TX mode, false for RX mode
 * @return 0 on success, negative errno code on failure
 */
int lora_radio_config(bool tx)
{
    struct lora_modem_config config;

    /* Configure LoRa parameters from Kconfig */
    config.frequency = CONFIG_APP_LORA_FREQUENCY;
    
    // Map Kconfig int to LoRa bandwidth enum
    switch (CONFIG_APP_LORA_BANDWIDTH) {
        case 0: config.bandwidth = BW_125_KHZ; break;
        case 1: config.bandwidth = BW_250_KHZ; break;
        case 2: config.bandwidth = BW_500_KHZ; break;
        // Add other cases as defined by LoRa driver and Kconfig help text
        default: config.bandwidth = BW_125_KHZ; // Fallback
                 LOG_WRN("Invalid APP_LORA_BANDWIDTH %d, defaulting to BW_125_KHZ", CONFIG_APP_LORA_BANDWIDTH);
                 break; 
    }

    // Map Kconfig int to LoRa spreading factor enum/define
    // Note: Zephyr's LoRa API might expect direct SF values (7-12) or specific defines.
    // Assuming direct SF values for datarate based on common practice.
    // If SF_X defines are needed, this mapping will be more complex.
    if (CONFIG_APP_LORA_SPREADING_FACTOR >= 5 && CONFIG_APP_LORA_SPREADING_FACTOR <= 12) {
        config.datarate = CONFIG_APP_LORA_SPREADING_FACTOR;
    } else {
        config.datarate = 7; // Default to SF7 if out of range
        LOG_WRN("Invalid APP_LORA_SPREADING_FACTOR %d, defaulting to SF7", CONFIG_APP_LORA_SPREADING_FACTOR);
    }
    
    config.preamble_len = CONFIG_APP_LORA_PREAMBLE_LEN;

    // Map Kconfig int to LoRa coding rate enum
    switch (CONFIG_APP_LORA_CODING_RATE) {
        case 1: config.coding_rate = CR_4_5; break;
        case 2: config.coding_rate = CR_4_6; break;
        case 3: config.coding_rate = CR_4_7; break;
        case 4: config.coding_rate = CR_4_8; break;
        default: config.coding_rate = CR_4_5; // Fallback
                 LOG_WRN("Invalid APP_LORA_CODING_RATE %d, defaulting to CR_4_5", CONFIG_APP_LORA_CODING_RATE);
                 break;
    }
    
    config.iq_inverted = IS_ENABLED(CONFIG_APP_LORA_IQ_INVERTED);
    config.public_network = IS_ENABLED(CONFIG_APP_LORA_PUBLIC_NETWORK);
    config.tx_power = CONFIG_APP_LORA_TX_POWER;
    config.tx = tx;                  /* TX or RX mode */

    /* Apply configuration to the radio */
    int ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        LOG_ERR("LoRa config failed: %d", ret);
        return ret;
    }

    /* If in RX mode, enable receive callbacks */
    if (tx == false) {
        ret = lora_recv_async(lora_dev, lora_rx_cb, NULL);
        if (ret < 0) {
            LOG_ERR("Failed to set LoRa receive callback: %d", ret);
            return ret;
        }
    }

    LOG_DBG("Radio configured for %s mode", tx ? "TX" : "RX");
    return 0;
}

/**
 * @brief Initialize the LoRa radio
 *
 * Configures the LoRa radio with default settings and
 * sets up the work queue for asynchronous transmission.
 *
 * @return 0 on success, negative errno code on failure
 */
int lora_init(void)
{
    /* Check if the LoRa device is ready */
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device '%s' not ready", lora_dev->name);
        return -ENODEV;
    }

    /* Initialize the work item for packet transmission */
    k_work_init(&lora_tx_work, lora_tx_send);
    
    /* Initialize the signal for asynchronous TX completion */
    k_poll_signal_init(&lora_tx_signal);

    /* Configure the radio for RX mode */
    int ret = lora_radio_config(false);
    if (ret < 0) {
        LOG_ERR("Failed to configure LoRa radio: %d", ret);
        return ret;
    }

    LOG_INF("LoRa radio initialized (node address: 0x%02X)", NODE_ADDRESS);
    return 0;
}
