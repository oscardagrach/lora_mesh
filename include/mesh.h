/**
 * @file mesh.h
 * @brief Mesh network management for LoRa communication
 *
 * This module handles mesh network functionality including node discovery,
 * neighbor management, and routing of messages between nodes. Supports
 * multi-hop message routing and reliable delivery with adaptive timeouts
 * and reliability-based route selection.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef MESH_H
#define MESH_H
 
#include <stdint.h>

/**
 * @brief Initialize the mesh network module
 *
 * Sets up the necessary work items for periodic mesh operations
 * such as neighbor discovery, maintenance, and pruning.
 */
void mesh_init(void);

/**
 * @brief Process a received mesh packet
 *
 * Handles different types of mesh packets (discovery, reply, data, etc.)
 * and updates the neighbor list accordingly. Implements multi-hop routing
 * for packets that need to be forwarded.
 *
 * @param buf Pointer to the packet buffer (including header)
 */
void mesh_process(uint8_t *buf);

/**
 * @brief Send a data packet through the mesh network
 *
 * Sends a data packet to a destination node, using multi-hop routing
 * if the destination is not a direct neighbor.
 *
 * @param dst Final destination node address
 * @param data Pointer to the data to send
 * @param len Length of the data in bytes
 * @return 0 on success, negative errno code on failure
 */
int mesh_send_data(uint32_t dst, const uint8_t *data, size_t len);

/**
 * @brief Send a broadcast data packet through the mesh network
 *
 * Sends a broadcast data packet to all neighbors in a single transmission.
 * Each neighbor that receives the packet will send an ACK back to the source.
 * This is more efficient than sending individual packets to each neighbor.
 *
 * @param data Pointer to the data to send
 * @param len Length of the data in bytes
 * @return 0 on success, negative errno code on failure
 */
int mesh_send_broadcast(const uint8_t *data, size_t len);

/**
 * @brief Find the best next hop for a destination
 *
 * Determines the best next hop to reach a destination node
 * based on the current routing information, signal quality,
 * and neighbor reliability metrics. Routes are scored based
 * on a combination of signal strength, reliability, and hop count.
 *
 * @param dst Destination node address
 * @return Next hop node address, or 0 if no route is available
 */
uint32_t mesh_find_next_hop(uint32_t dst);

/**
 * @brief Send an acknowledgment for a received packet
 *
 * Sends an acknowledgment packet back to the source of a received packet.
 *
 * @param orig_src Original source of the packet to acknowledge
 * @param seq Sequence number of the packet to acknowledge
 */
void mesh_send_ack(uint32_t orig_src, uint8_t seq);

/**
 * @brief Add a new neighbor or update an existing one
 *
 * Adds a new node to the neighbor list or updates its information
 * if it already exists. For new neighbors, initializes reliability
 * metrics (packet count, failed count, and reliability score) used
 * for adaptive timeout calculation and route selection.
 *
 * @param address Node address to add or update
 * @param rssi Signal strength of the received packet
 * @return Pointer to the neighbor structure, or NULL if failed
 */
struct neighbor *add_or_update_neighbor(uint32_t address, int16_t rssi);

/**
 * @brief Update the last seen timestamp and reliability metrics for a neighbor
 *
 * Updates the last seen timestamp for an existing neighbor to prevent it
 * from being pruned during maintenance. Also updates packet count and
 * reliability metrics used for adaptive timeout calculation and route selection.
 *
 * @param address Node address to update
 * @param rssi Signal strength of the received packet
 */
void update_neighbor_last_seen(uint32_t address, int16_t rssi);

#endif /* MESH_H */
