/**
 * @file lora.h
 * @brief LoRa radio interface for mesh network communication
 *
 * This module provides functions to initialize and control the LoRa radio
 * for transmitting and receiving packets in the mesh network.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef LORA_H
#define LORA_H
 
#include <zephyr/drivers/lora.h>
#include <zephyr/sys/atomic.h>
#include <packet.h>

// TX_POWER and SPREAD_FACTOR will now be sourced from Kconfig options
// (CONFIG_APP_LORA_TX_POWER and CONFIG_APP_LORA_SPREADING_FACTOR)
// and used directly in lora.c

/**
 * @brief Dynamic node address derived from device ID
 * 
 * The node address is dynamically generated from the last 4 bytes
 * of the device's unique hardware ID to ensure each device has
 * a unique address in the mesh network.
 */
extern uint32_t node_address;

/**
 * @brief Macro for backward compatibility
 * 
 * This macro allows existing code to continue using NODE_ADDRESS
 * while actually referencing the dynamic node_address variable.
 */
#define NODE_ADDRESS node_address

/**
 * @brief Counter for dropped packets due to buffer overflow
 */
extern atomic_t drop_pkt;

/**
 * @brief Initialize the LoRa radio
 *
 * Configures the LoRa radio with default settings and
 * sets up the work queue for asynchronous transmission.
 *
 * @return 0 on success, negative errno code on failure
 */
int lora_init(void);

/**
 * @brief Transmit a packet over LoRa
 *
 * Queues a packet for transmission over the LoRa radio.
 * The transmission is handled asynchronously by a work queue.
 *
 * @param buf Pointer to the packet buffer
 * @param size Size of the packet in bytes
 */
void lora_tx(uint8_t *buf, uint8_t size);

/**
 * @brief Configure the LoRa radio for TX or RX
 *
 * Sets the LoRa radio parameters for either transmission or reception.
 *
 * @param tx True for TX mode, false for RX mode
 * @return 0 on success, negative errno code on failure
 */
int lora_radio_config(bool tx);
 
#endif /* LORA_H */
