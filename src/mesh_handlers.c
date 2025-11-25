/**
 * @file mesh_handlers.c
 * @brief Focused mesh packet handlers for improved maintainability
 *
 * This module contains decomposed mesh packet handling functions,
 * breaking down the monolithic mesh_process() function into
 * focused, single-responsibility handlers.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <string.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mesh_handlers);

#include <lora.h>
#include <mesh.h>
#include <packet.h>
#include <sensor.h>
#include <config.h>
#include <mesh_utils.h>

/* Forward declarations for external functions */
extern void mesh_send_ack(uint32_t orig_src, uint8_t seq);
extern int forward_packet(uint8_t *buf);

/**
 * @brief Handle MESH_DISCOVER packets
 *
 * @param hdr Pointer to packet header
 * @param mesh Pointer to mesh payload
 */
static void handle_mesh_discover(const hdr_t *hdr, const mesh_t *mesh)
{
    LOG_INF("Received discovery request from 0x%08X", hdr->src);
    
    /* Send reply after a small delay to avoid collisions */
    hdr_t reply_hdr = {0};
    mesh_t reply_mesh = {0};
    uint8_t reply_packet[MAX_PACKET_LEN] = {0};

    /* Prepare reply packet header */
    reply_hdr.type = TYPE_MESH;
    reply_hdr.size = sizeof(hdr_t) + sizeof(mesh_t);
    reply_hdr.src = NODE_ADDRESS;
    reply_hdr.dst = hdr->src;

    /* Prepare reply mesh payload */
    reply_mesh.type = MESH_REPLY;

    /* Assemble reply packet */
    memcpy(reply_packet, &reply_hdr, sizeof(hdr_t));
    memcpy(reply_packet + sizeof(hdr_t), &reply_mesh, sizeof(mesh_t));

    /* Add a delay proportional to our address to avoid collisions */
    /* Use only the lower 8 bits of the 32-bit address to keep delay reasonable */
    k_msleep(RADIO_DISCOVERY_REPLY_DELAY_BASE_MS * (NODE_ADDRESS & 0xFF));

    LOG_INF("Sending discovery reply to 0x%08X", hdr->src);
    lora_tx(reply_packet, reply_hdr.size);
}

/**
 * @brief Handle MESH_REPLY packets
 *
 * @param hdr Pointer to packet header
 * @param mesh Pointer to mesh payload
 */
static void handle_mesh_reply(const hdr_t *hdr, const mesh_t *mesh)
{
    LOG_INF("Received discovery reply from 0x%08X (RSSI: %d)", 
            hdr->src, hdr->rssi);
}

/**
 * @brief Handle MESH_DATA packets for final destination
 *
 * @param buf Pointer to complete packet buffer
 * @param hdr Pointer to packet header
 * @param mesh Pointer to mesh payload
 */
static void handle_mesh_data_final(uint8_t *buf, const hdr_t *hdr, const mesh_t *mesh)
{
    LOG_INF("Received data packet from 0x%08X (seq %d, hops %d)", 
            mesh->orig_src, mesh->seq, hdr->hop_count);
    
    /* Send acknowledgment */
    mesh_send_ack(mesh->orig_src, mesh->seq);
    
    /* Process the data based on content */
    uint16_t payload_size = hdr->size - sizeof(hdr_t) - sizeof(mesh_t);
    
    /* Check if this is sensor data by verifying if the payload size is a multiple of sensor_t */
    if (payload_size > 0 && payload_size % sizeof(sensor_t) == 0) {
        LOG_INF("Detected sensor data in mesh packet (%u bytes, %u readings)", 
                payload_size, payload_size / sizeof(sensor_t));
        
        /* Process as sensor data - this will handle the logging with proper sensor names */
        sensor_process(buf);
    } else {
        /* Unknown data type, just log it */
        LOG_INF("Received unknown data type in mesh packet: %u bytes", payload_size);
    }
}

/**
 * @brief Handle MESH_DATA packets
 *
 * @param buf Pointer to complete packet buffer
 * @param hdr Pointer to packet header
 * @param mesh Pointer to mesh payload
 */
static void handle_mesh_data(uint8_t *buf, const hdr_t *hdr, const mesh_t *mesh)
{
    /* Check if we are the final destination */
    if (mesh->final_dst == NODE_ADDRESS) {
        handle_mesh_data_final(buf, hdr, mesh);
    } else {
        /* Forward the packet */
        LOG_INF("Forwarding data packet from 0x%08X to 0x%08X", 
                mesh->orig_src, mesh->final_dst);
        forward_packet(buf);
    }
}

/**
 * @brief Handle MESH_ACK packets for final destination
 *
 * @param hdr Pointer to packet header
 * @param mesh Pointer to mesh payload
 */
static void handle_mesh_ack_final(const hdr_t *hdr, const mesh_t *mesh)
{
    extern bool process_broadcast_ack(uint32_t src, uint8_t seq);
    extern bool remove_pending_packet(uint8_t seq, uint32_t dst);
    extern atomic_t acks_received;
    
    LOG_INF("Received ACK from 0x%08X for seq %d", 
            mesh->orig_src, mesh->seq);
    
    /* Update statistics */
    atomic_inc(&acks_received);
    
    /* Check if this is an ACK for a broadcast packet */
    if (process_broadcast_ack(mesh->orig_src, mesh->seq)) {
        LOG_INF("Processed ACK for broadcast packet seq %d from 0x%08X", 
                mesh->seq, mesh->orig_src);
    } 
    /* Otherwise, check if it's for a regular packet */
    else if (remove_pending_packet(mesh->seq, mesh->orig_src)) {
        LOG_INF("Successfully processed ACK for seq %d from 0x%08X", 
                mesh->seq, mesh->orig_src);
    } else {
        LOG_WRN("ACK received for unknown packet: seq %d from 0x%08X", 
                mesh->seq, mesh->orig_src);
    }
}

/**
 * @brief Handle MESH_ACK packets
 *
 * @param buf Pointer to complete packet buffer
 * @param hdr Pointer to packet header
 * @param mesh Pointer to mesh payload
 */
static void handle_mesh_ack(uint8_t *buf, const hdr_t *hdr, const mesh_t *mesh)
{
    /* Check if we are the final destination */
    if (mesh->final_dst == NODE_ADDRESS) {
        handle_mesh_ack_final(hdr, mesh);
    } else {
        /* Forward the ACK */
        LOG_INF("Forwarding ACK from 0x%08X to 0x%08X", 
                mesh->orig_src, mesh->final_dst);
        forward_packet(buf);
    }
}

/**
 * @brief Handle MESH_BROADCAST packets
 *
 * @param buf Pointer to complete packet buffer
 * @param hdr Pointer to packet header
 * @param mesh Pointer to mesh payload
 */
static void handle_mesh_broadcast(uint8_t *buf, const hdr_t *hdr, const mesh_t *mesh)
{
    LOG_INF("Received broadcast packet from 0x%08X (seq %d, hops %d)", 
            mesh->orig_src, mesh->seq, hdr->hop_count);
    
    /* Send acknowledgment back to the source */
    mesh_send_ack(mesh->orig_src, mesh->seq);
    
    /* Process the data based on content */
    uint16_t broadcast_payload_size = hdr->size - sizeof(hdr_t) - sizeof(mesh_t);
    
    /* Check if this is sensor data by verifying if the payload size is a multiple of sensor_t */
    if (broadcast_payload_size > 0 && broadcast_payload_size % sizeof(sensor_t) == 0) {
        LOG_INF("Detected sensor data in broadcast packet (%u bytes, %u readings)", 
                broadcast_payload_size, broadcast_payload_size / sizeof(sensor_t));
        
        /* Process as sensor data */
        sensor_process(buf);
    } else {
        /* Unknown data type, just log it */
        LOG_INF("Received unknown data type in broadcast packet: %u bytes", broadcast_payload_size);
    }
}

/**
 * @brief Validate incoming mesh packet
 *
 * @param buf Pointer to packet buffer
 * @param hdr Pointer to packet header
 * @return true if packet is valid, false otherwise
 */
static bool validate_incoming_packet(const uint8_t *buf, const hdr_t *hdr)
{
    /* Basic NULL check */
    if (buf == NULL || hdr == NULL) {
        LOG_ERR("NULL buffer or header provided");
        return false;
    }

    /* Validate source address */
    if (hdr->src == 0) {
        LOG_ERR("Invalid source address in mesh packet");
        return false;
    }

    /* Ignore our own packets that might be relayed back */
    if (hdr->src == NODE_ADDRESS) {
        LOG_DBG("Ignoring our own mesh packet");
        return false;
    }

    /* Additional packet validation using utility function */
    if (validate_mesh_packet(buf, hdr->size) != MESH_OK) {
        LOG_ERR("Mesh packet validation failed");
        return false;
    }

    return true;
}

/**
 * @brief Decomposed mesh packet processor
 *
 * This replaces the monolithic mesh_process() function with
 * a clean dispatcher that delegates to focused handlers.
 *
 * @param buf Pointer to the packet buffer (including header)
 */
void mesh_process_decomposed(uint8_t *buf)
{
    /* Input validation */
    if (buf == NULL) {
        LOG_ERR("NULL buffer provided to mesh_process");
        return;
    }

    hdr_t *hdr = (hdr_t *)buf;
    mesh_t *mesh = (mesh_t *)(buf + sizeof(hdr_t));

    /* Validate packet */
    if (!validate_incoming_packet(buf, hdr)) {
        return;
    }

    /* Update statistics atomically */
    extern atomic_t packets_received;
    atomic_inc(&packets_received);

    /* Update neighbor information for the direct sender */
    extern struct neighbor *add_or_update_neighbor(uint32_t address, int16_t rssi);
    add_or_update_neighbor(hdr->src, hdr->rssi);

    /* Update route quality for the original source if this is a multi-hop packet */
    if (mesh->type == MESH_DATA || mesh->type == MESH_ACK) {
        extern void update_route_quality(uint32_t address, int16_t rssi);
        update_route_quality(mesh->orig_src, hdr->rssi);
    }

    /* Dispatch to appropriate handler based on mesh packet type */
    switch (mesh->type) {
        case MESH_DISCOVER:
            handle_mesh_discover(hdr, mesh);
            break;
            
        case MESH_REPLY:
            handle_mesh_reply(hdr, mesh);
            break;
            
        case MESH_DATA:
            handle_mesh_data(buf, hdr, mesh);
            break;
            
        case MESH_ACK:
            handle_mesh_ack(buf, hdr, mesh);
            break;
            
        case MESH_BROADCAST:
            handle_mesh_broadcast(buf, hdr, mesh);
            break;
            
        case MESH_ROUTE_REQ:
            LOG_WRN("Route request not implemented yet");
            break;
            
        case MESH_ROUTE_RESP:
            LOG_WRN("Route response not implemented yet");
            break;
            
        default:
            LOG_ERR("Invalid mesh packet type: %d", mesh->type);
            break;
    }
}
