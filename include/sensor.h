/**
 * @file sensor.h
 * @brief Sensor management for data collection and transmission
 *
 * This module handles sensor detection, data collection, and transmission
 * of sensor readings over the LoRa mesh network.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef SENSOR_H
#define SENSOR_H
 
#include <zephyr/drivers/sensor.h>
#include <packet.h>

/**
 * @brief Initialize the sensor subsystem
 *
 * Detects and registers all compatible sensors connected to the device.
 */
void sensor_init(void);

/**
 * @brief Broadcast sensor readings over the mesh network
 *
 * Collects readings from all registered sensors and sends them
 * using the mesh network's reliable delivery mechanism. This leverages
 * multi-hop routing, acknowledgments, and retransmissions for more
 * reliable sensor data delivery.
 */
void sensor_broadcast(void);

/**
 * @brief Process a received sensor packet
 *
 * Parses and displays sensor readings received from another node.
 *
 * @param buf Pointer to the packet buffer (including header)
 */
void sensor_process(uint8_t *buf);

/**
 * @brief Get the number of registered sensors
 *
 * @return Number of sensors that were successfully registered
 */
int sensor_count(void);
 
#endif /* SENSOR_H */
