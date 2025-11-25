/**
 * @file config.h
 * @brief Configuration constants for LoRa mesh network
 *
 * Centralized configuration to eliminate magic numbers and improve
 * maintainability across the codebase.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================== */
/* MESH NETWORK CONFIGURATION                                                 */
/* ========================================================================== */

/** @brief Maximum number of neighbors that can be tracked */
#define MESH_MAX_NEIGHBORS 32

/** @brief Maximum number of routes that can be stored */
#define MESH_MAX_ROUTES 32

/** @brief Maximum number of pending packets waiting for acknowledgment */
#define MESH_MAX_PENDING_PACKETS 16

/** @brief Maximum time to live (TTL) for packets */
#define MESH_MAX_TTL 10

/** @brief Base time in milliseconds after which a neighbor is considered stale */
#define MESH_PRUNE_TIMEOUT_BASE_MS 60000

/** @brief Maximum time in milliseconds for neighbor pruning */
#define MESH_PRUNE_TIMEOUT_MAX_MS 120000

/** @brief Minimum time in milliseconds for neighbor pruning */
#define MESH_PRUNE_TIMEOUT_MIN_MS 30000

/** @brief RSSI threshold below which a neighbor is considered weak */
#define MESH_WEAK_NEIGHBOR_RSSI_THRESHOLD -100

/** @brief Factor to reduce pruning timeout for weak neighbors */
#define MESH_WEAK_NEIGHBOR_TIMEOUT_FACTOR 0.5f

/** @brief Factor to increase pruning timeout for strong neighbors */
#define MESH_STRONG_NEIGHBOR_TIMEOUT_FACTOR 1.5f

/* ========================================================================== */
/* DISCOVERY CONFIGURATION                                                    */
/* ========================================================================== */

/** @brief Base interval in seconds between neighbor discovery broadcasts */
#define MESH_DISCOVERY_INTERVAL_BASE_SEC 60

/** @brief Minimum interval in seconds between neighbor discovery broadcasts */
#define MESH_DISCOVERY_INTERVAL_MIN_SEC 30

/** @brief Maximum interval in seconds between neighbor discovery broadcasts */
#define MESH_DISCOVERY_INTERVAL_MAX_SEC 300

/** @brief Step size in seconds for increasing discovery interval */
#define MESH_DISCOVERY_INTERVAL_STEP_SEC 30

/** @brief Window in milliseconds to consider topology changes recent */
#define MESH_TOPOLOGY_CHANGE_WINDOW_MS 120000

/** @brief Initial delay in seconds before first discovery broadcast */
#define MESH_INITIAL_DISCOVERY_DELAY_SEC 10

/* ========================================================================== */
/* TIMING CONFIGURATION                                                       */
/* ========================================================================== */

/** @brief Interval in seconds between neighbor list display */
#define MESH_NEIGHBOR_LIST_INTERVAL_SEC 30

/** @brief Interval in seconds between neighbor list pruning */
#define MESH_PRUNE_INTERVAL_SEC 60

/** @brief Interval in seconds between route table pruning */
#define MESH_ROUTE_PRUNE_INTERVAL_SEC 120

/** @brief Interval in seconds between network statistics display */
#define MESH_STATS_DISPLAY_INTERVAL_SEC 60

/* ========================================================================== */
/* RELIABILITY CONFIGURATION                                                  */
/* ========================================================================== */

/** @brief Maximum number of retransmission attempts */
#define MESH_MAX_RETRANSMISSIONS 5

/** @brief Base timeout in milliseconds for acknowledgment */
#define MESH_ACK_TIMEOUT_BASE_MS 10000

/** @brief Maximum timeout in milliseconds for acknowledgment */
#define MESH_ACK_TIMEOUT_MAX_MS 30000

/** @brief Minimum timeout in milliseconds for acknowledgment */
#define MESH_ACK_TIMEOUT_MIN_MS 5000

/** @brief Interval in milliseconds between retransmission attempts */
#define MESH_RETRANSMISSION_INTERVAL_BASE_MS 1000

/** @brief Factor to increase timeout per hop count */
#define MESH_TIMEOUT_HOP_FACTOR_MS 500

/** @brief Time in milliseconds after which a route is considered stale */
#define MESH_ROUTE_TIMEOUT_MS 120000

/** @brief Hysteresis threshold for route quality changes to prevent flapping */
#define MESH_ROUTE_QUALITY_HYSTERESIS 5

/* ========================================================================== */
/* SENSOR CONFIGURATION                                                       */
/* ========================================================================== */

/** @brief Maximum number of sensors that can be registered */
#define SENSOR_MAX_DEVICES 5

/** @brief Maximum number of channels per sensor */
#define SENSOR_MAX_CHANNELS 6

/** @brief Maximum number of sensor readings in a single packet */
#define SENSOR_MAX_READINGS (SENSOR_MAX_DEVICES * SENSOR_MAX_CHANNELS)

/** @brief Interval in milliseconds between sensor data broadcasts */
#define SENSOR_BROADCAST_INTERVAL_MS 10000

/* ========================================================================== */
/* RADIO CONFIGURATION                                                        */
/* ========================================================================== */

/** @brief LED pulse duration in milliseconds for radio activity */
#define RADIO_LED_PULSE_DURATION_MS 50

/** @brief Delay in milliseconds for discovery replies to avoid collisions */
#define RADIO_DISCOVERY_REPLY_DELAY_BASE_MS 200

/** @brief Delay in milliseconds for ACK transmission to avoid collisions */
#define RADIO_ACK_DELAY_MS 50

/* ========================================================================== */
/* BUFFER CONFIGURATION                                                       */
/* ========================================================================== */

/** @brief Maximum number of packets to process in a single work item execution */
#define PACKET_MAX_BATCH_SIZE 5

/* ========================================================================== */
/* VALIDATION MACROS                                                          */
/* ========================================================================== */

/** @brief Validate that a pointer is not NULL */
#define VALIDATE_PTR(ptr) \
    do { if (!(ptr)) { LOG_ERR("NULL pointer: " #ptr); return -EINVAL; } } while(0)

/** @brief Validate that a value is within a specified range */
#define VALIDATE_RANGE(val, min, max) \
    do { if ((val) < (min) || (val) > (max)) { \
        LOG_ERR("Value out of range: " #val "=%d (expected %d-%d)", \
                (int)(val), (int)(min), (int)(max)); \
        return -EINVAL; \
    } } while(0)

/** @brief Validate that an address is not zero (invalid) */
#define VALIDATE_ADDRESS(addr) \
    do { if ((addr) == 0) { \
        LOG_ERR("Invalid address: " #addr "=0x%02X", (addr)); \
        return -EINVAL; \
    } } while(0)

/* ========================================================================== */
/* ERROR CODES                                                                */
/* ========================================================================== */

/** @brief Mesh-specific error codes */
typedef enum {
    MESH_OK = 0,                    /**< Operation successful */
    MESH_ERR_INVALID_PARAM = -1,    /**< Invalid parameter */
    MESH_ERR_NO_MEMORY = -2,        /**< Out of memory */
    MESH_ERR_NO_ROUTE = -3,         /**< No route to destination */
    MESH_ERR_TIMEOUT = -4,          /**< Operation timed out */
    MESH_ERR_BUFFER_FULL = -5,      /**< Buffer is full */
    MESH_ERR_NOT_FOUND = -6,        /**< Item not found */
    MESH_ERR_INVALID_STATE = -7     /**< Invalid state for operation */
} mesh_error_t;

#endif /* CONFIG_H */
