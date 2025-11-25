/**
 * @file packet.h
 * @brief Packet handling for LoRa mesh network communication
 *
 * This module defines the packet structure and types used for communication
 * between nodes in the LoRa mesh network. It handles packet processing,
 * including receiving, parsing, and dispatching to appropriate handlers.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef PACKET_H
#define PACKET_H

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdint.h>
#include <sensor.h>

/**
 * @brief Maximum packet length in bytes
 * 
 * This defines the maximum size of a packet that can be transmitted or received.
 * Limited by LoRa radio capabilities.
 */
#define MAX_PACKET_LEN  255

/**
 * @brief Size of receive buffer in bytes
 */
#define RX_BUF_LEN 2048

/**
 * @brief Size of transmit buffer in bytes
 */
#define TX_BUF_LEN 2048

/**
 * @brief Work queue for packet processing
 * 
 * This work queue is used to process packets asynchronously
 * to avoid blocking the main thread or interrupt handlers.
 */
extern struct k_work_q packet_work_q;

/**
 * @brief Work item for packet processing
 * 
 * This work item is submitted to the packet_work_q when
 * new packets are available for processing.
 */
extern struct k_work packet_process_work;

/**
 * @brief Initialize the packet processing module
 *
 * Sets up the work queue and work item for packet processing.
 *
 * @return 0 on success, negative errno code on failure
 */
int packet_init(void);

/**
 * @brief Packet type enumeration
 * 
 * Defines the different types of packets that can be sent/received
 * in the mesh network.
 */
typedef enum {
    TYPE_SENSOR, /**< Sensor data packet */
    TYPE_FOTA,   /**< Firmware update packet */
    TYPE_MESH,   /**< Mesh network management packet */
    TYPE_MAX     /**< Maximum packet type value (for validation) */
} packet_type_t;

/**
 * @brief Mesh packet subtype enumeration
 * 
 * Defines the different types of mesh management packets.
 */
typedef enum {
    MESH_DISCOVER,   /**< Node discovery request */
    MESH_REPLY,      /**< Response to discovery request */
    MESH_DATA,       /**< Data packet to be routed */
    MESH_ACK,        /**< Acknowledgment of received packet */
    MESH_BROADCAST,  /**< Broadcast data packet */
    MESH_ROUTE_REQ,  /**< Route request */
    MESH_ROUTE_RESP, /**< Route response */
    MESH_MAX         /**< Maximum mesh type value (for validation) */
} mesh_type_t;

/**
 * @brief Mesh packet payload structure
 */
typedef struct {
    uint8_t type;       /**< Mesh packet subtype (mesh_type_t) */
    uint8_t seq;        /**< Sequence number for packet tracking */
    uint32_t orig_src;  /**< Original source address (for multi-hop) */
    uint32_t final_dst; /**< Final destination address (for multi-hop) */
    uint8_t ttl;        /**< Time to live (hop limit) */
    uint8_t payload[0]; /**< Variable-length payload */
} __attribute__ ((packed)) mesh_t;

/**
 * @brief Sensor data payload structure
 */
typedef struct {
    uint8_t chan;  /**< Sensor channel identifier */
    float value;  /**< Sensor reading value */
} __attribute__ ((packed)) sensor_t;

/**
 * @brief Packet header structure
 * 
 * This structure is prepended to all packets for routing and identification.
 */
typedef struct {
    uint16_t size;     /**< Total packet size in bytes (including header) */
    uint8_t type;      /**< Packet type (packet_type_t) */
    uint32_t src;      /**< Source node address (current hop) */
    uint32_t dst;      /**< Destination node address (next hop, 0 for broadcast) */
    uint16_t seq_num;  /**< Sequence number for packet tracking */
    uint8_t hop_count; /**< Number of hops traversed */
    int16_t rssi;      /**< Received signal strength indicator (dBm) */
    int8_t snr;        /**< Signal-to-noise ratio (dB) */
} __attribute__ ((packed)) hdr_t;

#endif /* PACKET_H */
