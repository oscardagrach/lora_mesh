/**
 * @file mesh_debug.h
 * @brief Debugging and monitoring tools for mesh network
 *
 * This module provides comprehensive debugging, monitoring, and
 * profiling capabilities for the mesh network system.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef MESH_DEBUG_H
#define MESH_DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include <packet.h>
#include <config.h>

/**
 * @brief Debug configuration flags
 */
#ifdef CONFIG_MESH_DEBUG
#define MESH_DEBUG_PACKET_TRACE 1
#define MESH_DEBUG_NEIGHBOR_CHANGES 1
#define MESH_DEBUG_ROUTE_CHANGES 1
#define MESH_DEBUG_PERFORMANCE_STATS 1
#else
#define MESH_DEBUG_PACKET_TRACE 0
#define MESH_DEBUG_NEIGHBOR_CHANGES 0
#define MESH_DEBUG_ROUTE_CHANGES 0
#define MESH_DEBUG_PERFORMANCE_STATS 0
#endif

/**
 * @brief Performance profiling structure
 */
typedef struct {
    /* Timing measurements */
    uint32_t packet_process_time_avg_us;  /**< Average packet processing time */
    uint32_t neighbor_lookup_time_avg_us; /**< Average neighbor lookup time */
    uint32_t route_lookup_time_avg_us;    /**< Average route lookup time */
    
    /* Memory usage */
    uint32_t neighbor_memory_used;        /**< Memory used by neighbor table */
    uint32_t route_memory_used;           /**< Memory used by routing table */
    uint32_t pending_memory_used;         /**< Memory used by pending packets */
    
    /* Performance counters */
    uint32_t neighbor_lookup_count;       /**< Number of neighbor lookups */
    uint32_t route_lookup_count;          /**< Number of route lookups */
    uint32_t packet_process_count;        /**< Number of packets processed */
    
    /* System health indicators */
    uint32_t mutex_contention_count;      /**< Number of mutex contentions */
    uint32_t memory_allocation_failures;  /**< Number of failed allocations */
    uint32_t work_queue_overruns;         /**< Number of work queue overruns */
} mesh_performance_t;

/**
 * @brief Initialize debugging and monitoring system
 */
void mesh_debug_init(void);

/**
 * @brief Dump neighbor table for debugging
 *
 * Prints detailed information about all neighbors including
 * signal quality, reliability metrics, and timing information.
 */
void debug_dump_neighbor_table(void);

/**
 * @brief Dump routing table for debugging
 *
 * Prints detailed information about all routes including
 * hop counts, quality metrics, and age information.
 */
void debug_dump_route_table(void);

/**
 * @brief Dump pending packet list for debugging
 *
 * Prints information about all pending packets waiting
 * for acknowledgment, including retry counts and timeouts.
 */
void debug_dump_pending_packets(void);

/**
 * @brief Trace packet flow through the system
 *
 * Logs packet movement through different stages of processing
 * for debugging network behavior.
 *
 * @param hdr Pointer to packet header
 * @param stage Description of processing stage
 */
void debug_trace_packet_flow(const hdr_t *hdr, const char *stage);

/**
 * @brief Log neighbor changes for debugging
 *
 * Tracks and logs neighbor additions, updates, and removals
 * to help debug network topology changes.
 *
 * @param address Neighbor address
 * @param action Description of the action (added, updated, removed)
 * @param rssi Signal strength
 */
void debug_log_neighbor_change(uint8_t address, const char *action, int16_t rssi);

/**
 * @brief Log route changes for debugging
 *
 * Tracks and logs route additions, updates, and removals
 * to help debug routing behavior.
 *
 * @param dst Destination address
 * @param next_hop Next hop address
 * @param action Description of the action
 * @param quality Route quality metric
 */
void debug_log_route_change(uint8_t dst, uint8_t next_hop, const char *action, int16_t quality);

/**
 * @brief Get current performance statistics
 *
 * Returns current performance metrics for monitoring
 * and optimization purposes.
 *
 * @param perf Pointer to performance structure to fill
 * @return 0 on success, negative error code on failure
 */
int debug_get_performance_stats(mesh_performance_t *perf);

/**
 * @brief Start timing measurement
 *
 * Begins timing measurement for performance profiling.
 * Use with debug_end_timing() to measure operation duration.
 *
 * @return Start timestamp
 */
static inline int64_t debug_start_timing(void) {
    return k_uptime_get();
}

/**
 * @brief End timing measurement and log result
 *
 * Ends timing measurement and logs the result if debugging is enabled.
 *
 * @param start_time Start timestamp from debug_start_timing()
 * @param operation_name Name of the operation being timed
 * @return Duration in microseconds
 */
uint32_t debug_end_timing(int64_t start_time, const char *operation_name);

/**
 * @brief Check system health and log warnings
 *
 * Monitors system health indicators and logs warnings
 * when thresholds are exceeded.
 */
void debug_check_system_health(void);

/**
 * @brief Reset performance counters
 *
 * Resets all performance counters and statistics to zero.
 * Useful for benchmarking specific scenarios.
 */
void debug_reset_performance_counters(void);

/**
 * @brief Validate system invariants
 *
 * Checks critical system invariants and logs errors
 * if any are violated. Helps detect corruption or bugs.
 *
 * @return true if all invariants are valid, false otherwise
 */
bool debug_validate_system_invariants(void);

/**
 * @brief Generate comprehensive system report
 *
 * Creates a detailed report of system state including
 * neighbor table, routing table, statistics, and health indicators.
 */
void debug_generate_system_report(void);

/**
 * @brief Debug assertion macro
 *
 * Logs assertion failures and optionally triggers system reset
 * in debug builds.
 */
#ifdef CONFIG_MESH_DEBUG
#define MESH_ASSERT(condition, msg) \
    do { if (!(condition)) { \
        LOG_ERR("ASSERTION FAILED in %s:%d: " msg, __FILE__, __LINE__); \
        debug_generate_system_report(); \
        /* Could trigger reset in production */ \
    } } while(0)
#else
#define MESH_ASSERT(condition, msg) do { } while(0)
#endif

/**
 * @brief Debug trace macro for packet flow
 */
#if MESH_DEBUG_PACKET_TRACE
#define DEBUG_TRACE_PACKET(hdr, stage) debug_trace_packet_flow(hdr, stage)
#else
#define DEBUG_TRACE_PACKET(hdr, stage) do { } while(0)
#endif

/**
 * @brief Debug log macro for neighbor changes
 */
#if MESH_DEBUG_NEIGHBOR_CHANGES
#define DEBUG_LOG_NEIGHBOR(addr, action, rssi) debug_log_neighbor_change(addr, action, rssi)
#else
#define DEBUG_LOG_NEIGHBOR(addr, action, rssi) do { } while(0)
#endif

/**
 * @brief Debug log macro for route changes
 */
#if MESH_DEBUG_ROUTE_CHANGES
#define DEBUG_LOG_ROUTE(dst, next_hop, action, quality) debug_log_route_change(dst, next_hop, action, quality)
#else
#define DEBUG_LOG_ROUTE(dst, next_hop, action, quality) do { } while(0)
#endif

#endif /* MESH_DEBUG_H */
