/**
 * @file mesh_utils.c
 * @brief Implementation of utility functions for mesh network operations
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <string.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mesh_utils);

#include <mesh_utils.h>
#include <lora.h>

/**
 * @brief Check if a packet is destined for this node
 *
 * @param hdr Pointer to packet header
 * @return true if packet is for this node, false otherwise
 */
bool is_packet_for_us(const hdr_t *hdr)
{
    if (hdr == NULL) {
        return false;
    }
    
    mesh_t *mesh = (mesh_t *)((uint8_t *)hdr + sizeof(hdr_t));
    
    /* Check if it's a direct packet to us */
    if (hdr->dst == NODE_ADDRESS) {
        return true;
    }
    
    /* Check if it's a mesh packet with us as final destination */
    if (hdr->type == TYPE_MESH && mesh->final_dst == NODE_ADDRESS) {
        return true;
    }
    
    /* Check if it's a broadcast packet (dst == 0) */
    if (hdr->dst == 0) {
        return true;
    }
    
    return false;
}

/**
 * @brief Calculate adaptive timeout based on reliability and hop count
 *
 * @param reliability Neighbor reliability score (0-100)
 * @param hop_count Number of hops to destination
 * @return Calculated timeout in milliseconds
 */
uint32_t calculate_adaptive_timeout(uint8_t reliability, uint8_t hop_count)
{
    uint32_t timeout = MESH_ACK_TIMEOUT_BASE_MS;
    
    /* Adjust for hop count */
    timeout += (hop_count - 1) * MESH_TIMEOUT_HOP_FACTOR_MS;
    
    /* Adjust for reliability */
    if (reliability < 50) {
        /* Low reliability - increase timeout */
        timeout = (uint32_t)(timeout * 1.5f);
    } else if (reliability > 90) {
        /* High reliability - decrease timeout */
        timeout = (uint32_t)(timeout * 0.8f);
    }
    
    /* Ensure timeout is within bounds */
    return clamp_value(timeout, MESH_ACK_TIMEOUT_MIN_MS, MESH_ACK_TIMEOUT_MAX_MS);
}

/**
 * @brief Log packet information for debugging
 *
 * @param prefix Prefix string for the log message
 * @param hdr Pointer to packet header
 */
void log_packet_info(const char *prefix, const hdr_t *hdr)
{
    if (hdr == NULL || prefix == NULL) {
        return;
    }
    
    LOG_DBG("%s: size=%u, type=%u, src=0x%08X, dst=0x%08X, seq=%u, hops=%u, RSSI=%d",
            prefix, hdr->size, hdr->type, hdr->src, hdr->dst,
            hdr->seq_num, hdr->hop_count, hdr->rssi);
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
mesh_error_t validate_mesh_packet(const uint8_t *buf, size_t size)
{
    if (buf == NULL) {
        return MESH_ERR_INVALID_PARAM;
    }
    
    if (size < sizeof(hdr_t)) {
        return MESH_ERR_INVALID_PARAM;
    }
    
    const hdr_t *hdr = (const hdr_t *)buf;
    
    /* Validate packet size */
    if (hdr->size != size) {
        return MESH_ERR_INVALID_PARAM;
    }
    
    /* Validate packet type */
    if (hdr->type >= TYPE_MAX) {
        return MESH_ERR_INVALID_PARAM;
    }
    
    /* If it's a mesh packet, validate mesh header */
    if (hdr->type == TYPE_MESH) {
        if (size < sizeof(hdr_t) + sizeof(mesh_t)) {
            return MESH_ERR_INVALID_PARAM;
        }
        
        const mesh_t *mesh = (const mesh_t *)(buf + sizeof(hdr_t));
        
        /* Validate mesh type */
        if (mesh->type >= MESH_MAX) {
            return MESH_ERR_INVALID_PARAM;
        }
        
        /* Validate TTL */
        if (mesh->ttl == 0 || mesh->ttl > MESH_MAX_TTL) {
            return MESH_ERR_INVALID_PARAM;
        }
    }
    
    return MESH_OK;
}

/**
 * @brief Get mesh packet type safely
 *
 * Extracts mesh packet type with bounds checking.
 *
 * @param buf Pointer to packet buffer
 * @return Mesh packet type, or MESH_MAX on error
 */
uint8_t get_mesh_type(const uint8_t *buf)
{
    if (buf == NULL) {
        return MESH_MAX;
    }
    
    const hdr_t *hdr = (const hdr_t *)buf;
    
    if (hdr->type != TYPE_MESH) {
        return MESH_MAX;
    }
    
    const mesh_t *mesh = (const mesh_t *)(buf + sizeof(hdr_t));
    
    if (mesh->type >= MESH_MAX) {
        return MESH_MAX;
    }
    
    return mesh->type;
}

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
int32_t calculate_route_score(int16_t rssi, uint8_t reliability, uint8_t hop_count)
{
    int32_t score = rssi;
    
    /* Adjust for reliability - scale from 0-100 to -50 to +50 adjustment */
    int32_t reliability_adjustment = ((int32_t)reliability - 50);
    score += reliability_adjustment;
    
    /* Penalize for hop count - each hop reduces score */
    score -= (hop_count - 1) * 10;
    
    return score;
}

/**
 * @brief Add jitter to timeout value
 *
 * Adds random jitter to prevent synchronized retransmissions.
 *
 * @param base_timeout Base timeout value in milliseconds
 * @param jitter_percent Jitter percentage (0-100)
 * @return Timeout with jitter applied
 */
uint32_t add_timeout_jitter(uint32_t base_timeout, uint8_t jitter_percent)
{
    if (jitter_percent == 0 || jitter_percent > 100) {
        return base_timeout;
    }
    
    /* Calculate maximum jitter amount */
    uint32_t max_jitter = (base_timeout * jitter_percent) / 100;
    
    /* Generate random jitter (±max_jitter/2) */
    int32_t jitter = (k_uptime_get() % max_jitter) - (max_jitter / 2);
    
    /* Apply jitter, ensuring result is positive */
    int32_t result = (int32_t)base_timeout + jitter;
    if (result <= 0) {
        result = base_timeout / 2;
    }
    
    return (uint32_t)result;
}
