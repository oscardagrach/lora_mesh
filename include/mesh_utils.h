/**
 * @file mesh_utils.h
 * @brief Utility functions for mesh network operations
 *
 * This module provides common utility functions used across the mesh
 * network implementation to reduce code duplication and improve
 * maintainability.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <packet.h>
#include <config.h>

/**
 * @brief Check if an address is valid (non-zero)
 *
 * @param addr Address to validate
 * @return true if address is valid, false otherwise
 */
static inline bool is_valid_address(uint8_t addr) {
    return addr != 0;
}

/**
 * @brief Check if a packet is destined for this node
 *
 * @param hdr Pointer to packet header
 * @return true if packet is for this node, false otherwise
 */
bool is_packet_for_us(const hdr_t *hdr);

/**
 * @brief Calculate adaptive timeout based on reliability and hop count
 *
 * @param reliability Neighbor reliability score (0-100)
 * @param hop_count Number of hops to destination
 * @return Calculated timeout in milliseconds
 */
uint32_t calculate_adaptive_timeout(uint8_t reliability, uint8_t hop_count);

/**
 * @brief Log packet information for debugging
 *
 * @param prefix Prefix string for the log message
 * @param hdr Pointer to packet header
 */
void log_packet_info(const char *prefix, const hdr_t *hdr);

/**
 * @brief Calculate hash value for neighbor address
 *
 * Simple hash function for distributing neighbors across hash table.
 *
 * @param addr Node address to hash
 * @return Hash value (0 to MESH_NEIGHBOR_HASH_SIZE-1)
 */
static inline uint8_t neighbor_hash_func(uint8_t addr) {
    return addr % MESH_MAX_NEIGHBORS;
}

/**
 * @brief Calculate hash value for route destination
 *
 * Simple hash function for distributing routes across hash table.
 *
 * @param dst Destination address to hash
 * @return Hash value (0 to MESH_ROUTE_HASH_SIZE-1)
 */
static inline uint8_t route_hash_func(uint8_t dst) {
    return dst % MESH_MAX_ROUTES;
}

/**
 * @brief Validate mesh packet structure
 *
 * Performs basic validation of mesh packet headers and payload.
 *
 * @param buf Pointer to packet buffer
 * @param size Size of the packet buffer
 * @return MESH_OK on success, negative error code on failure
 */
mesh_error_t validate_mesh_packet(const uint8_t *buf, size_t size);

/**
 * @brief Get mesh packet type safely
 *
 * Extracts mesh packet type with bounds checking.
 *
 * @param buf Pointer to packet buffer
 * @return Mesh packet type, or MESH_MAX on error
 */
uint8_t get_mesh_type(const uint8_t *buf);

/**
 * @brief Calculate route quality score
 *
 * Combines RSSI, reliability, and hop count into a single score.
 *
 * @param rssi Signal strength
 * @param reliability Neighbor reliability (0-100)
 * @param hop_count Number of hops
 * @return Route quality score (higher is better)
 */
int32_t calculate_route_score(int16_t rssi, uint8_t reliability, uint8_t hop_count);

/**
 * @brief Add jitter to timeout value
 *
 * Adds random jitter to prevent synchronized retransmissions.
 *
 * @param base_timeout Base timeout value in milliseconds
 * @param jitter_percent Jitter percentage (0-100)
 * @return Timeout with jitter applied
 */
uint32_t add_timeout_jitter(uint32_t base_timeout, uint8_t jitter_percent);

/**
 * @brief Check if two timestamps indicate a recent change
 *
 * @param current_time Current timestamp
 * @param last_change Timestamp of last change
 * @param window_ms Time window in milliseconds
 * @return true if change was recent, false otherwise
 */
static inline bool is_recent_change(int64_t current_time, int64_t last_change, uint32_t window_ms) {
    return (current_time - last_change) < window_ms;
}

/**
 * @brief Clamp a value between minimum and maximum bounds
 *
 * @param value Value to clamp
 * @param min_val Minimum allowed value
 * @param max_val Maximum allowed value
 * @return Clamped value
 */
static inline uint32_t clamp_value(uint32_t value, uint32_t min_val, uint32_t max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

#endif /* MESH_UTILS_H */
