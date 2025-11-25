/**
 * @file packet.c
 * @brief Packet handling for LoRa mesh network communication
 *
 * This module implements packet processing functionality, including
 * receiving, parsing, and dispatching packets to appropriate handlers.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/reboot.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(packet);

#include <lora.h>
#include <mesh.h>
#include <ring_buffer.h>
#include <sensor.h>
#include <packet.h>
#include <work_queue.h>
#include <config.h>
#include <mesh_utils.h>

/**
 * @brief Stack size for the packet processing work queue
 */
#define STACK_SIZE  4096  /* Increased from 2048 to prevent stack overflow */

/**
 * @brief Priority for the packet processing work queue (higher number = lower priority)
 */
#define WORK_QUEUE_PRIORITY 6

/* Work item for packet processing */
struct k_work packet_process_work;

/* Packet handler function type definition */
typedef void (*packet_handler_t)(uint8_t *buf);

/* Packet handler lookup table for faster dispatch */
static const packet_handler_t packet_handlers[] = {
    sensor_process, /* TYPE_SENSOR */
    NULL,           /* TYPE_FOTA - not implemented yet */
    mesh_process    /* TYPE_MESH */
};

/**
 * @brief Dump packet contents for debugging
 *
 * Prints the raw bytes of a packet in hexadecimal format.
 *
 * @param buf Pointer to the packet buffer
 */
static void packet_dump(uint8_t *buf)
{
    if (buf == NULL) {
        LOG_ERR("NULL buffer provided to packet_dump");
        return;
    }

    for (int i = 0; i < MAX_PACKET_LEN; i++) {
        if (i % 16 == 0) {
            printk("\n");
        }
        printk("%02hhX ", buf[i]);
    }
    printk("\n");
}

/**
 * @brief Maximum number of packets to process in a single work item execution
 */
#define MAX_PACKETS_PER_WORK_ITEM PACKET_MAX_BATCH_SIZE

/**
 * @brief Process packets from the RX ring buffer
 *
 * This function is called as a work item when new packets are available
 * in the RX ring buffer. It reads packets from the buffer and dispatches
 * them to the appropriate handler based on the packet type. Limits the
 * number of packets processed per invocation to prevent starvation of
 * other tasks.
 *
 * @param work Pointer to the work item
 */
static void packet_process(struct k_work *work)
{
    int ret = 0;
    uint8_t buf[MAX_PACKET_LEN] = {0};
    hdr_t *hdr = (hdr_t *)buf;
    size_t rx_size;
    int packets_processed = 0;

    /* Process a limited number of packets per work item invocation */
    while ((rx_size = rb_size_get_rx()) >= sizeof(hdr_t) && 
           packets_processed < MAX_PACKETS_PER_WORK_ITEM) {
        /* First peek at the header to determine the full packet size */
        ret = rb_peek_rx(buf, sizeof(hdr_t));
        
        /* Validate header */
        if (ret != sizeof(hdr_t)) {
            LOG_ERR("Failed to peek packet header: got %d bytes", ret);
            break;
        }
        
        /* Validate packet size */
        if (hdr->size < sizeof(hdr_t) || hdr->size > MAX_PACKET_LEN) {
            LOG_ERR("Invalid packet size in header: %u", hdr->size);
            /* Discard the invalid header to avoid getting stuck */
            rb_read_rx(buf, sizeof(hdr_t));
            continue;
        }
        
        /* Check if we have the complete packet */
        if (rx_size < hdr->size) {
            /* Not enough data for a complete packet, wait for more */
            break;
        }
        
        /* Read the complete packet */
        ret = rb_read_rx(buf, hdr->size);
        if (ret != hdr->size) {
            LOG_ERR("Failed to read packet: expected %u bytes, got %d", 
                    hdr->size, ret);
            break;
        }

        /* Log packet information (reduced verbosity for better performance) */
        LOG_DBG("Packet: size=%u, type=%u, src=0x%08X, dst=0x%08X, RSSI=%d",
                hdr->size, hdr->type, hdr->src, hdr->dst, hdr->rssi);

        /* Update neighbor information */
        update_neighbor_last_seen(hdr->src, hdr->rssi);

        /* Dispatch packet to appropriate handler based on type */
        if (hdr->type < TYPE_MAX && packet_handlers[hdr->type] != NULL) {
            packet_handlers[hdr->type](buf);
        } else {
            LOG_WRN("Unknown packet type: %u", hdr->type);
            packet_dump(buf);
        }
        
        packets_processed++;
    }
    
    /* If there are more packets to process, resubmit the work item */
    if (rb_size_get_rx() >= sizeof(hdr_t)) {
        k_work_submit_to_queue(&app_work_q, &packet_process_work);
    }
}

/**
 * @brief Initialize the packet processing module
 *
 * Sets up the work item for packet processing.
 *
 * @return 0 on success, negative errno code on failure
 */
int packet_init(void)
{
    /* Initialize the work item for packet processing */
    k_work_init(&packet_process_work, packet_process);

    LOG_INF("Packet processing initialized");
    return 0;
}
