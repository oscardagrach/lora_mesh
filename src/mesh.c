/**
 * @file mesh.c
 * @brief Mesh network management for LoRa communication
 *
 * This module implements mesh network functionality including node discovery,
 * neighbor management, and routing of messages between nodes.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <string.h>

#include <lora.h>
#include <mesh.h>
#include <packet.h>
#include <work_queue.h>
#include <config.h>
#include <mesh_utils.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mesh);

/* All configuration constants now sourced from config.h */

/* Work items for periodic mesh operations */
static struct k_work_delayable discover_work;
static struct k_work_delayable neighbors_list_work;
static struct k_work_delayable prune_work;
static struct k_work_delayable route_prune_work;

/* Work item for deferred route creation */
static struct k_work add_route_work;

/* Data for deferred route creation */
static struct {
    uint32_t address;
    int16_t rssi;
} route_work_data;

/**
 * @brief Neighbor node information structure
 */
typedef struct neighbor {
    uint32_t address;          /**< Node address */
    int16_t rssi;              /**< Signal strength of last communication */
    int64_t last_seen;         /**< Timestamp of last communication */
    uint8_t is_direct;         /**< Whether this is a direct neighbor (1) or multi-hop (0) */
    uint16_t packet_count;     /**< Number of packets received from this neighbor */
    uint16_t failed_count;     /**< Number of failed transmissions to this neighbor */
    uint8_t reliability;       /**< Reliability score (0-100) */
    struct neighbor *next;     /**< Pointer to next neighbor in linked list */
} neighbor_t;

/**
 * @brief Routing table entry structure
 */
typedef struct route {
    uint32_t dst;              /**< Destination node address */
    uint32_t next_hop;         /**< Next hop node address */
    uint8_t hop_count;         /**< Number of hops to destination */
    int16_t quality;           /**< Route quality metric (based on RSSI) */
    int64_t last_updated;      /**< Timestamp of last route update */
    struct route *next;        /**< Pointer to next route in linked list */
} route_t;

/**
 * @brief Pending packet structure for reliable delivery
 */
typedef struct pending_packet {
    uint32_t dst;              /**< Destination node address */
    uint8_t seq;               /**< Sequence number */
    uint8_t retries;           /**< Number of retransmission attempts */
    int64_t sent_time;         /**< Timestamp when packet was sent */
    uint8_t data[MAX_PACKET_LEN]; /**< Packet data */
    uint16_t size;             /**< Packet size */
    struct k_work_delayable retry_work; /**< Work item for retransmission */
    struct pending_packet *next; /**< Pointer to next pending packet */
} pending_packet_t;

/**
 * @brief Broadcast ACK tracking structure
 * 
 * Used to track which neighbors have acknowledged a broadcast packet.
 */
typedef struct broadcast_ack_tracker {
    uint8_t seq;                /**< Sequence number of the broadcast packet */
    uint8_t retries;            /**< Number of retransmission attempts */
    int64_t sent_time;          /**< Timestamp when packet was sent */
    uint8_t data[MAX_PACKET_LEN]; /**< Packet data */
    uint16_t size;              /**< Packet size */
    struct k_work_delayable retry_work; /**< Work item for retransmission */
    
    uint8_t expected_acks;      /**< Number of expected ACKs */
    uint8_t received_acks;      /**< Number of received ACKs */
    uint32_t neighbor_addrs[MESH_MAX_NEIGHBORS]; /**< Addresses of neighbors expected to ACK */
    bool neighbor_acked[MESH_MAX_NEIGHBORS];     /**< Whether each neighbor has ACKed */
    
    struct broadcast_ack_tracker *next;    /**< Pointer to next tracker */
} broadcast_ack_tracker_t;

/* Head of the broadcast ACK tracker list */
static broadcast_ack_tracker_t *broadcast_tracker_list = NULL;

/* Mutex for broadcast tracker list protection */
K_MUTEX_DEFINE(broadcast_tracker_mutex);

/* Memory slab for broadcast ACK trackers */
K_MEM_SLAB_DEFINE(broadcast_tracker_slab, sizeof(broadcast_ack_tracker_t), MESH_MAX_NEIGHBORS, 4);

/* Memory slabs for efficient allocation/deallocation */
K_MEM_SLAB_DEFINE(neighbor_slab, sizeof(neighbor_t), MESH_MAX_NEIGHBORS, 4);
K_MEM_SLAB_DEFINE(route_slab, sizeof(route_t), MESH_MAX_ROUTES, 4);
K_MEM_SLAB_DEFINE(pending_packet_slab, sizeof(pending_packet_t), MESH_MAX_PENDING_PACKETS, 4);

/* Mutexes for list protection */
K_MUTEX_DEFINE(neighbor_list_mutex);
K_MUTEX_DEFINE(route_list_mutex);
K_MUTEX_DEFINE(pending_packet_list_mutex);

/* Heads of the linked lists */
static neighbor_t *neighbor_list = NULL;
static route_t *route_list = NULL;
static pending_packet_t *pending_packet_list = NULL;

/* Sequence number for outgoing packets */
static uint8_t next_seq = 1;

/**
 * @brief Interval in seconds between network statistics display
 */
#define STATS_DISPLAY_INTERVAL_SEC 60

/* Atomic statistics counters for thread safety and performance */
static atomic_t neighbor_changes = ATOMIC_INIT(0);
static atomic_t packets_sent = ATOMIC_INIT(0);
static atomic_t packets_received = ATOMIC_INIT(0);
static atomic_t packets_forwarded = ATOMIC_INIT(0);
static atomic_t packets_dropped = ATOMIC_INIT(0);
static atomic_t acks_sent = ATOMIC_INIT(0);
static atomic_t acks_received = ATOMIC_INIT(0);
static atomic_t retransmissions = ATOMIC_INIT(0);

/* Non-atomic statistics (updated less frequently) */
typedef struct {
    int64_t last_topology_change;  /**< Timestamp of last topology change */
    uint32_t current_discovery_interval; /**< Current discovery interval in seconds */
    int64_t last_stats_time;       /**< Timestamp of last statistics calculation */
    double packets_per_minute;     /**< Packet rate (packets per minute) */
    double delivery_success_rate;  /**< Percentage of successful deliveries */
} network_stats_t;

static network_stats_t stats = {
    .last_topology_change = 0,
    .current_discovery_interval = MESH_DISCOVERY_INTERVAL_BASE_SEC,
    .last_stats_time = 0,
    .packets_per_minute = 0.0,
    .delivery_success_rate = 100.0
};

/* Mutex for non-atomic statistics */
K_MUTEX_DEFINE(stats_mutex);

/* Work item for statistics display */
static struct k_work_delayable stats_work;

/* Forward declarations */
static bool neighbor_change_detected(void);
static void mesh_retry_handler(struct k_work *work);
static route_t *add_or_update_route(uint32_t dst, uint32_t next_hop, uint8_t hop_count, int16_t quality);
static void mesh_stats_display(struct k_work *work);
static void update_route_quality(uint32_t address, int16_t rssi);
static void route_prune_work_handler(struct k_work *work);
static bool process_broadcast_ack(uint32_t src, uint8_t seq);
static void add_route_work_handler(struct k_work *work);

/**
 * @brief Add a new neighbor or update an existing one
 *
 * Adds a new node to the neighbor list or updates its information
 * if it already exists.
 *
 * @param address Node address to add or update
 * @param rssi Signal strength of the received packet
 * @return Pointer to the neighbor structure, or NULL if failed
 */
neighbor_t *add_or_update_neighbor(uint32_t address, int16_t rssi) {
    int64_t now = k_uptime_get();
    neighbor_t *current;
    neighbor_t *new_neighbor = NULL;
    neighbor_t *result = NULL;

    /* Ignore invalid addresses */
    if (address == 0) {
        LOG_WRN("Ignoring invalid neighbor address 0");
        return NULL;
    }

    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);

    current = neighbor_list;
    /* Check if neighbor already exists */
    while (current != NULL) {
        if (current->address == address) {
            current->rssi = rssi;
            current->last_seen = now;
            current->is_direct = 1; /* Mark as direct neighbor */
            LOG_DBG("Updated neighbor 0x%08X, RSSI: %d", address, rssi);
            result = current;
            goto unlock_and_return_neighbor; // Changed label
        }
        current = current->next;
    }

    /* Allocate new neighbor from slab */
    if (k_mem_slab_alloc(&neighbor_slab, (void **)&new_neighbor, K_NO_WAIT) == 0) {
        new_neighbor->address = address;
        new_neighbor->rssi = rssi;
        new_neighbor->last_seen = now;
        new_neighbor->is_direct = 1; /* Mark as direct neighbor */
        new_neighbor->packet_count = 1; /* Initialize packet count */
        new_neighbor->failed_count = 0; /* Initialize failed count */
        new_neighbor->reliability = 100; /* Initialize reliability score */
        new_neighbor->next = neighbor_list;
        neighbor_list = new_neighbor;
        result = new_neighbor;
        LOG_INF("Added new neighbor 0x%08X, RSSI: %d", address, rssi);
        
        /* Track topology change for adaptive discovery */
        atomic_inc(&neighbor_changes);
        k_mutex_lock(&stats_mutex, K_FOREVER);
        stats.last_topology_change = now;
        k_mutex_unlock(&stats_mutex);
        LOG_DBG("Topology change: new neighbor added");
        
        /* Schedule deferred route creation for new neighbor */
        route_work_data.address = address;
        route_work_data.rssi = rssi;
        k_work_submit_to_queue(&app_work_q, &add_route_work);
        
    } else {
        LOG_ERR("Neighbor list full (%d nodes), unable to add 0x%08X", 
                MESH_MAX_NEIGHBORS, address);
        result = NULL;
    }

unlock_and_return_neighbor: // Changed label
    k_mutex_unlock(&neighbor_list_mutex);
    return result;
}

/**
 * @brief Add or update a route in the routing table
 *
 * @param dst Destination node address
 * @param next_hop Next hop node address
 * @param hop_count Number of hops to destination
 * @param quality Route quality metric (based on RSSI)
 * @return Pointer to the route structure, or NULL if failed
 */
static route_t *add_or_update_route(uint32_t dst, uint32_t next_hop, uint8_t hop_count, int16_t quality) {
    int64_t now = k_uptime_get();
    route_t *current;
    route_t *new_route = NULL;
    route_t *result = NULL;

    /* Ignore invalid addresses */
    if (dst == 0 || next_hop == 0) {
        LOG_WRN("Ignoring invalid route with dst=0x%02X, next_hop=0x%02X", dst, next_hop);
        return NULL;
    }

    k_mutex_lock(&route_list_mutex, K_FOREVER);

    current = route_list;
    /* Check if route already exists */
    while (current != NULL) {
        if (current->dst == dst) {
            /* For direct neighbors (hop_count=1), always update the timestamp */
            if (hop_count == 1) {
                current->last_updated = now;
                
                /* Update other fields only if the new route is better */
                if (hop_count < current->hop_count || 
                    (hop_count == current->hop_count && quality > current->quality)) {
                    current->next_hop = next_hop;
                    current->hop_count = hop_count;
                    current->quality = quality;
                    LOG_DBG("Updated route to 0x%02X via 0x%02X, hops: %d, quality: %d", 
                            dst, next_hop, hop_count, quality);
                } else {
                    LOG_DBG("Refreshed route timestamp to 0x%02X", dst);
                }
            } else {
                /* For multi-hop routes, only update if the new route is better */
                if (hop_count < current->hop_count || 
                    (hop_count == current->hop_count && quality > current->quality)) {
                    current->next_hop = next_hop;
                    current->hop_count = hop_count;
                    current->quality = quality;
                    current->last_updated = now;
                    LOG_DBG("Updated route to 0x%02X via 0x%02X, hops: %d, quality: %d", 
                            dst, next_hop, hop_count, quality);
                }
            }
            result = current;
            goto unlock_and_return_route;
        }
        current = current->next;
    }

    /* Allocate new route from slab */
    if (k_mem_slab_alloc(&route_slab, (void **)&new_route, K_NO_WAIT) == 0) {
        new_route->dst = dst;
        new_route->next_hop = next_hop;
        new_route->hop_count = hop_count;
        new_route->quality = quality;
        new_route->last_updated = now;
        new_route->next = route_list;
        route_list = new_route;
        result = new_route;
        LOG_INF("Added new route to 0x%02X via 0x%02X, hops: %d, quality: %d", 
                dst, next_hop, hop_count, quality);
    } else {
        LOG_ERR("Route table full (%d routes), unable to add route to 0x%02X", 
                MESH_MAX_ROUTES, dst);
        result = NULL;
    }

unlock_and_return_route:
    k_mutex_unlock(&route_list_mutex);
    return result;
}


/**
 * @brief Find the best next hop for a destination
 *
 * Determines the best next hop to reach a destination node
 * based on the current routing information and neighbor reliability.
 *
 * @param dst Destination node address
 * @return Next hop node address, or 0 if no route is available
 */
uint32_t mesh_find_next_hop(uint32_t dst) {
    route_t *current_route;
    neighbor_t *current_neighbor;
    uint32_t next_hop_addr = 0;
    int32_t best_score = INT32_MIN;
    
    /* If destination is ourselves, return our address */
    if (dst == NODE_ADDRESS) {
        return NODE_ADDRESS;
    }

    /* First, find all valid routes to the destination */
    k_mutex_lock(&route_list_mutex, K_FOREVER);
    current_route = route_list;
    
    /* Get current time once for all comparisons */
    int64_t now = k_uptime_get();
    
    /* First pass: find all valid routes and their scores */
    route_t *best_route = NULL;
    while (current_route != NULL) {
        if (current_route->dst == dst) {
            /* Check if the route is still valid */
            if ((now - current_route->last_updated) <= MESH_ROUTE_TIMEOUT_MS) {
                /* Find the neighbor for this route's next hop */
                k_mutex_unlock(&route_list_mutex); // Unlock before accessing neighbor_list
                k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
                
                uint8_t neighbor_reliability = 0;
                current_neighbor = neighbor_list;
                while (current_neighbor != NULL) {
                    if (current_neighbor->address == current_route->next_hop) {
                        neighbor_reliability = current_neighbor->reliability;
                        break;
                    }
                    current_neighbor = current_neighbor->next;
                }
                
                k_mutex_unlock(&neighbor_list_mutex);
                k_mutex_lock(&route_list_mutex, K_FOREVER); // Re-lock route_list_mutex
                
                /* Calculate score for this route */
                int32_t score = calculate_route_score(
                    current_route->quality, 
                    neighbor_reliability, 
                    current_route->hop_count
                );
                
                LOG_DBG("Route to 0x%02X via 0x%02X: quality=%d, reliability=%u%%, hops=%d, score=%d",
                        dst, current_route->next_hop, current_route->quality, 
                        neighbor_reliability, current_route->hop_count, score);
                
                /* Keep track of the best route */
                if (score > best_score) {
                    best_score = score;
                    best_route = current_route;
                }
            } else {
                LOG_WRN("Route to 0x%02X via 0x%02X is stale, last updated %lld ms ago", 
                        dst, current_route->next_hop, (now - current_route->last_updated));
            }
        }
        current_route = current_route->next;
    }
    
    /* If we found a valid route, use its next hop */
    if (best_route != NULL) {
        next_hop_addr = best_route->next_hop;
        LOG_DBG("Selected best route to 0x%02X via 0x%02X (score: %d)", 
                dst, next_hop_addr, best_score);
    }
    
    k_mutex_unlock(&route_list_mutex);

    if (next_hop_addr != 0) {
        return next_hop_addr;
    }
    
    /* No route found or existing route was stale, check if it's a direct neighbor */
    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    current_neighbor = neighbor_list;
    while (current_neighbor != NULL) {
        if (current_neighbor->address == dst && current_neighbor->is_direct) {
            /* It's a direct neighbor, add a route and return */
            /* add_or_update_route handles its own mutex for route_list */
            k_mutex_unlock(&neighbor_list_mutex); // Unlock before calling add_or_update_route
            add_or_update_route(dst, dst, 1, current_neighbor->rssi);
            return dst; // Return dst as next hop for direct neighbor
        }
        current_neighbor = current_neighbor->next;
    }
    k_mutex_unlock(&neighbor_list_mutex);
    
    /* No route available */
    LOG_WRN("No route available to 0x%02X", dst);
    return 0;
}

/**
 * @brief Update route quality based on received packet RSSI
 *
 * Updates the quality of routes to a node based on the RSSI of received packets.
 * Uses exponential moving average to smooth out fluctuations and applies
 * hysteresis to prevent route flapping.
 *
 * @param address Node address to update route quality for
 * @param rssi Signal strength of the received packet
 */
static void update_route_quality(uint32_t address, int16_t rssi)
{
    route_t *current_route;
    neighbor_t *next_hop_neighbor;
    int64_t now = k_uptime_get();
    
    /* Ignore invalid addresses */
    if (address == 0 || address == NODE_ADDRESS) {
        return;
    }

    k_mutex_lock(&route_list_mutex, K_FOREVER);
    current_route = route_list;
    
    /* Find routes to this destination */
    while (current_route != NULL) {
        if (current_route->dst == address) {
            /* Skip stale routes */
            if ((now - current_route->last_updated) > MESH_ROUTE_TIMEOUT_MS) {
                current_route = current_route->next;
                continue;
            }
            
            /* For direct routes, quality is directly based on RSSI */
            if (current_route->hop_count == 1) {
                /* Calculate new quality with exponential moving average */
                int16_t new_quality = (current_route->quality * 3 + rssi) / 4;
                
                /* Only update if the change is significant (hysteresis) */
                if (abs(new_quality - current_route->quality) > MESH_ROUTE_QUALITY_HYSTERESIS) {
                    current_route->quality = new_quality;
                    current_route->last_updated = now;
                    LOG_DBG("Updated route quality to 0x%02X: %d -> %d", 
                            address, current_route->quality, new_quality);
                }
            } 
            /* For multi-hop routes, quality is affected by the first hop's RSSI */
            else {
                k_mutex_unlock(&route_list_mutex); // Unlock before accessing neighbor_list
                k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
                next_hop_neighbor = neighbor_list;
                while (next_hop_neighbor != NULL) {
                    if (next_hop_neighbor->address == current_route->next_hop) {
                        break;
                    }
                    next_hop_neighbor = next_hop_neighbor->next;
                }
                k_mutex_unlock(&neighbor_list_mutex);
                k_mutex_lock(&route_list_mutex, K_FOREVER); // Re-lock route_list_mutex
                
                /* If next hop is a direct neighbor, update route quality */
                if (next_hop_neighbor != NULL && next_hop_neighbor->is_direct) {
                    /* Calculate new quality based on next hop's RSSI */
                    int16_t new_quality = next_hop_neighbor->rssi - 
                                         (5 * (current_route->hop_count - 1)); /* Penalty for each hop */
                    
                    /* Only update if the change is significant (hysteresis) */
                    if (abs(new_quality - current_route->quality) > MESH_ROUTE_QUALITY_HYSTERESIS) {
                        current_route->quality = new_quality;
                        current_route->last_updated = now;
                        LOG_DBG("Updated multi-hop route quality to 0x%02X via 0x%02X: %d -> %d", 
                                address, current_route->next_hop, current_route->quality, new_quality);
                    }
                }
            }
        }
        current_route = current_route->next;
    }
    k_mutex_unlock(&route_list_mutex);
}

/**
 * @brief Update the last seen timestamp for a neighbor
 *
 * Updates the last seen timestamp for an existing neighbor
 * to prevent it from being pruned during maintenance.
 * Also updates the corresponding route to prevent it from becoming stale.
 *
 * @param address Node address to update
 * @param rssi Signal strength of the received packet
 */
void update_neighbor_last_seen(uint32_t address, int16_t rssi)
{
    neighbor_t *current;
    int64_t now = k_uptime_get();
    bool found = false;
    int16_t cached_rssi = 0;
    bool is_direct_neighbor = false;

    /* Ignore invalid addresses */
    if (address == 0) {
        return;
    }

    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    current = neighbor_list;
    while (current != NULL) {
        if (current->address == address) {
            current->last_seen = now;
            
            /* Update RSSI with exponential moving average */
            current->rssi = (current->rssi * 3 + rssi) / 4;
            cached_rssi = current->rssi;
            
            /* Update packet count and reliability metrics */
            current->packet_count++;
            
            /* Calculate reliability score (0-100) */
            if (current->packet_count > 0 && current->failed_count > 0) {
                current->reliability = 100 - (current->failed_count * 100) / 
                                     (current->packet_count + current->failed_count);
            } else {
                current->reliability = 100; /* Perfect reliability if no failures */
            }
            
            LOG_DBG("Updated neighbor 0x%02X metrics: packets=%u, failures=%u, reliability=%u%%", 
                    address, current->packet_count, current->failed_count, current->reliability);
            
            /* Cache direct neighbor status */
            is_direct_neighbor = current->is_direct;
            found = true;
            break;
        }
        current = current->next;
    }
    k_mutex_unlock(&neighbor_list_mutex);

    /* Update the direct route if neighbor was found and is direct */
    if (found && is_direct_neighbor) {
        add_or_update_route(address, address, 1, cached_rssi);
    }

    /* If not found and it's a valid address, add it as a new neighbor */
    if (!found && address != 0 && address != NODE_ADDRESS) {
        add_or_update_neighbor(address, rssi);
    }
}

/**
 * @brief Calculate adaptive timeout for neighbor pruning
 *
 * Calculates a timeout value based on neighbor signal quality.
 * Strong neighbors get longer timeouts, weak ones get shorter.
 *
 * @param neighbor Pointer to the neighbor structure
 * @return Timeout in milliseconds
 */
static uint32_t calculate_neighbor_timeout(neighbor_t *neighbor)
{
    uint32_t timeout = MESH_PRUNE_TIMEOUT_BASE_MS;
    float reliability_factor = 1.0f;
    
    /* Adjust based on signal strength */
    if (neighbor->rssi < MESH_WEAK_NEIGHBOR_RSSI_THRESHOLD) {
        /* Weak signal - reduce timeout */
        timeout = (uint32_t)(timeout * MESH_WEAK_NEIGHBOR_TIMEOUT_FACTOR);
    } else if (neighbor->rssi > (MESH_WEAK_NEIGHBOR_RSSI_THRESHOLD + 20)) {
        /* Strong signal - increase timeout */
        timeout = (uint32_t)(timeout * MESH_STRONG_NEIGHBOR_TIMEOUT_FACTOR);
    }
    
    /* Adjust based on reliability */
    if (neighbor->packet_count > 5) {  /* Only consider reliability after sufficient data */
        if (neighbor->reliability < 50) {
            /* Low reliability - reduce timeout */
            reliability_factor = 0.5f;
        } else if (neighbor->reliability > 90) {
            /* High reliability - increase timeout */
            reliability_factor = 1.5f;
        } else {
            /* Scale linearly between 0.5 and 1.5 based on reliability */
            reliability_factor = 0.5f + (neighbor->reliability / 100.0f);
        }
        
        timeout = (uint32_t)(timeout * reliability_factor);
        
        LOG_DBG("Neighbor 0x%02X timeout adjusted by reliability factor %.2f (reliability: %u%%)",
                neighbor->address, (double)reliability_factor, neighbor->reliability);
    }
    
    /* Ensure timeout is within bounds */
    timeout = MIN(timeout, MESH_PRUNE_TIMEOUT_MAX_MS);
    timeout = MAX(timeout, MESH_PRUNE_TIMEOUT_MIN_MS);
    
    return timeout;
}

/**
 * @brief Remove stale neighbors from the list
 *
 * Removes neighbors that haven't been seen for longer than
 * their adaptive timeout from the neighbor list.
 */
static void prune_neighbors(void) {
    int64_t now = k_uptime_get();
    neighbor_t **current;
    int pruned = 0;

    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    current = &neighbor_list;
    while (*current != NULL) {
        /* Calculate adaptive timeout for this neighbor */
        uint32_t timeout = calculate_neighbor_timeout(*current);
        
        if ((now - (*current)->last_seen) > timeout) {
            neighbor_t *expired = *current;
            *current = expired->next;
            LOG_INF("Pruning stale neighbor 0x%02X, last seen %lld ms ago (timeout: %u ms, RSSI: %d)", 
                    expired->address, (now - expired->last_seen), timeout, expired->rssi);
            k_mem_slab_free(&neighbor_slab, (void *)expired);
            pruned++;
        } else {
            current = &(*current)->next;
        }
    }
    
    if (pruned > 0) {
        LOG_INF("Pruned %d stale neighbors", pruned);
        
        /* Track topology change for adaptive discovery */
        atomic_add(&neighbor_changes, pruned);
        k_mutex_lock(&stats_mutex, K_FOREVER);
        stats.last_topology_change = k_uptime_get();
        k_mutex_unlock(&stats_mutex);
        LOG_DBG("Topology change: %d neighbors pruned", pruned);
    }
    k_mutex_unlock(&neighbor_list_mutex);
}

/**
 * @brief Remove stale routes from the routing table
 *
 * Removes routes that haven't been updated for longer than
 * ROUTE_TIMEOUT_MS from the routing table.
 */
static void prune_routes(void) {
    int64_t now = k_uptime_get();
    route_t **current;
    int pruned = 0;

    k_mutex_lock(&route_list_mutex, K_FOREVER);
    current = &route_list;
    while (*current != NULL) {
        if ((now - (*current)->last_updated) > MESH_ROUTE_TIMEOUT_MS) {
            route_t *expired = *current;
            *current = expired->next;
            LOG_INF("Pruning stale route to 0x%02X via 0x%02X, last updated %lld ms ago", 
                    expired->dst, expired->next_hop, (now - expired->last_updated));
            k_mem_slab_free(&route_slab, (void *)expired);
            pruned++;
        } else {
            current = &(*current)->next;
        }
    }
    
    if (pruned > 0) {
        LOG_INF("Pruned %d stale routes", pruned);
        
        /* Track topology change for adaptive discovery */
        atomic_add(&neighbor_changes, pruned);
        k_mutex_lock(&stats_mutex, K_FOREVER);
        stats.last_topology_change = k_uptime_get();
        k_mutex_unlock(&stats_mutex);
        LOG_DBG("Topology change: %d routes pruned", pruned);
    }
    k_mutex_unlock(&route_list_mutex);
}

/**
 * @brief Handler for the route prune work item
 *
 * Prunes stale routes and reschedules itself.
 *
 * @param work Pointer to the work item
 */
static void route_prune_work_handler(struct k_work *work)
{
    prune_routes();
    k_work_schedule(&route_prune_work, K_SECONDS(MESH_ROUTE_PRUNE_INTERVAL_SEC));
}

/**
 * @brief Handle retransmission of unacknowledged broadcast packets
 *
 * @param work Pointer to the work item
 */
static void mesh_broadcast_retry_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    broadcast_ack_tracker_t *tracker = CONTAINER_OF(dwork, broadcast_ack_tracker_t, retry_work);
    bool all_acked = true;
    int missing_acks = 0;
    
    k_mutex_lock(&broadcast_tracker_mutex, K_FOREVER);
    
    /* Check if all neighbors have acknowledged */
    for (int i = 0; i < tracker->expected_acks; i++) {
        if (!tracker->neighbor_acked[i]) {
            all_acked = false;
            missing_acks++;
        }
    }
    
    if (all_acked) {
        LOG_INF("All %d neighbors have acknowledged broadcast packet seq %d", 
                tracker->expected_acks, tracker->seq);
        
        /* Remove from tracker list */
        broadcast_ack_tracker_t **current = &broadcast_tracker_list;
        while (*current != NULL) {
            if (*current == tracker) {
                *current = tracker->next;
                k_mem_slab_free(&broadcast_tracker_slab, tracker);
                break;
            }
            current = &(*current)->next;
        }
        
        k_mutex_unlock(&broadcast_tracker_mutex);
        return;
    }
    
    /* Check if max retries reached */
    if (tracker->retries >= MESH_MAX_RETRANSMISSIONS) {
        LOG_WRN("Max retries reached for broadcast packet seq %d, %d/%d neighbors acknowledged", 
                tracker->seq, tracker->received_acks, tracker->expected_acks);
        
        /* Update reliability metrics for neighbors that didn't ACK */
        k_mutex_unlock(&broadcast_tracker_mutex); // Unlock before accessing neighbor_list
        k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
        
        for (int i = 0; i < tracker->expected_acks; i++) {
            if (!tracker->neighbor_acked[i]) {
                uint32_t addr = tracker->neighbor_addrs[i];
                neighbor_t *neighbor = neighbor_list;
                
                while (neighbor != NULL) {
                    if (neighbor->address == addr) {
                        neighbor->failed_count++;
                        
                        /* Recalculate reliability score */
                        if (neighbor->packet_count > 0) {
                            neighbor->reliability = 100 - (neighbor->failed_count * 100) / 
                                                 (neighbor->packet_count + neighbor->failed_count);
                        }
                        
                        LOG_WRN("Updated reliability for neighbor 0x%02X: %u%% (failures: %u, packets: %u)",
                                neighbor->address, neighbor->reliability, 
                                neighbor->failed_count, neighbor->packet_count);
                        break;
                    }
                    neighbor = neighbor->next;
                }
            }
        }
        
        k_mutex_unlock(&neighbor_list_mutex);
        k_mutex_lock(&broadcast_tracker_mutex, K_FOREVER); // Re-lock for tracker list operations
        
        /* Remove from tracker list */
        broadcast_ack_tracker_t **current = &broadcast_tracker_list;
        while (*current != NULL) {
            if (*current == tracker) {
                *current = tracker->next;
                k_mem_slab_free(&broadcast_tracker_slab, tracker);
                break;
            }
            current = &(*current)->next;
        }
        
        k_mutex_unlock(&broadcast_tracker_mutex);
        return;
    }
    
    /* Increment retry count */
    tracker->retries++;
    tracker->sent_time = k_uptime_get();
    
    /* Update statistics */
    atomic_inc(&retransmissions);
    
    LOG_INF("Retrying broadcast packet seq %d (attempt %d/%d), waiting for %d more ACKs",
            tracker->seq, tracker->retries, MESH_MAX_RETRANSMISSIONS, missing_acks);
    
    /* Retransmit the packet */
    lora_tx(tracker->data, tracker->size);
    
    /* Calculate timeout for next retry */
    uint32_t timeout = MESH_ACK_TIMEOUT_BASE_MS;
    
    /* Add randomization to avoid collisions (±20% of timeout) */
    int32_t jitter = (k_uptime_get() % (timeout / 5)) - (timeout / 10);
    timeout += jitter;
    
    LOG_DBG("Scheduling broadcast retry with timeout %u ms (including jitter %d ms)", 
            timeout, jitter);
    
    /* Schedule next retry */
    k_work_schedule(&tracker->retry_work, K_MSEC(timeout));
    
    k_mutex_unlock(&broadcast_tracker_mutex);
}

/**
 * @brief Handler for the prune work item
 *
 * Prunes stale neighbors and reschedules itself.
 *
 * @param work Pointer to the work item
 */
static void prune_work_handler(struct k_work *work)
{
    prune_neighbors();
    k_work_schedule(&prune_work, K_SECONDS(MESH_PRUNE_INTERVAL_SEC));
}

/**
 * @brief Send a mesh discovery packet
 *
 * Broadcasts a discovery packet to find other nodes in the network.
 *
 * @param work Pointer to the work item
 */
/**
 * @brief Check if there have been recent topology changes
 *
 * Determines if there have been any neighbor additions or removals
 * within the topology change window.
 *
 * @return true if topology changes detected, false otherwise
 */
static bool neighbor_change_detected(void)
{
    int64_t now = k_uptime_get();
    
    /* Check if there have been neighbor changes within the window */
    k_mutex_lock(&stats_mutex, K_FOREVER);
    bool changes_detected = (atomic_get(&neighbor_changes) > 0 && 
        (now - stats.last_topology_change) < MESH_TOPOLOGY_CHANGE_WINDOW_MS);
    k_mutex_unlock(&stats_mutex);
    
    return changes_detected;
}

/**
 * @brief Send a mesh discovery packet
 *
 * Broadcasts a discovery packet to find other nodes in the network.
 * Uses adaptive discovery interval based on network stability.
 *
 * @param work Pointer to the work item
 */
static void mesh_discover(struct k_work *work)
{
    hdr_t hdr = {0};
    mesh_t mesh = {0};
    uint8_t packet[MAX_PACKET_LEN] = {0};

    /* Prepare packet header */
    hdr.type = TYPE_MESH;
    hdr.size = sizeof(hdr_t) + sizeof(mesh_t);
    hdr.src = NODE_ADDRESS;
    hdr.dst = 0; /* Broadcast */

    /* Prepare mesh payload */
    mesh.type = MESH_DISCOVER;

    /* Assemble packet */
    memcpy(packet, &hdr, sizeof(hdr_t));
    memcpy(packet + sizeof(hdr_t), &mesh, sizeof(mesh_t));

    LOG_INF("Broadcasting mesh discovery packet");
    lora_tx(packet, hdr.size);

    /* Adjust discovery interval based on network stability */
    if (neighbor_change_detected()) {
        /* Network topology changing - discover more frequently */
        stats.current_discovery_interval = MAX(MESH_DISCOVERY_INTERVAL_MIN_SEC,
                                              stats.current_discovery_interval / 2);
        LOG_INF("Network topology changing, reducing discovery interval to %d seconds",
                stats.current_discovery_interval);
    } else {
        /* Network stable - gradually increase interval */
        stats.current_discovery_interval = MIN(MESH_DISCOVERY_INTERVAL_MAX_SEC,
                                              stats.current_discovery_interval + MESH_DISCOVERY_INTERVAL_STEP_SEC);
        LOG_INF("Network stable, increasing discovery interval to %d seconds",
                stats.current_discovery_interval);
    }

    /* Reset neighbor change counter */
    atomic_set(&neighbor_changes, 0);

    /* Schedule next discovery with adaptive interval */
    k_work_schedule(&discover_work, K_SECONDS(stats.current_discovery_interval));
}

/**
 * @brief Send a reply to a discovery packet
 *
 * Sends a directed reply to a node that sent a discovery packet.
 *
 * @param dst Destination node address
 */
static void mesh_reply(uint32_t dst)
{
    hdr_t hdr = {0};
    mesh_t mesh = {0};
    uint8_t packet[MAX_PACKET_LEN] = {0};

    /* Prepare packet header */
    hdr.type = TYPE_MESH;
    hdr.size = sizeof(hdr_t) + sizeof(mesh_t);
    hdr.src = NODE_ADDRESS;
    hdr.dst = dst;

    /* Prepare mesh payload */
    mesh.type = MESH_REPLY;

    /* Assemble packet */
    memcpy(packet, &hdr, sizeof(hdr_t));
    memcpy(packet + sizeof(hdr_t), &mesh, sizeof(mesh_t));

    /* Add a delay proportional to our address to avoid collisions */
    /* Use only the lower 8 bits of the 32-bit address to keep delay reasonable */
    k_msleep(200 * (NODE_ADDRESS & 0xFF));

    LOG_INF("Sending discovery reply to 0x%02X", dst);
    lora_tx(packet, hdr.size);
}

/**
 * @brief Display the current neighbor list and routing table
 *
 * Logs the current list of neighbors and routes with their information.
 *
 * @param work Pointer to the work item
 */
static void mesh_neighbor_list(struct k_work *work)
{
    neighbor_t *current_neighbor;
    route_t *current_route;
    int index = 0;
    int64_t now = k_uptime_get();

    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    current_neighbor = neighbor_list;
    /* Display neighbor list */
    if (!current_neighbor) {
        LOG_INF("Neighbor list is empty");
    } else {
        LOG_INF("=== Neighbor List ===");
        while (current_neighbor != NULL) {
            int64_t age = now - current_neighbor->last_seen;

            LOG_INF("[%d] Addr: 0x%02X | RSSI: %d | Direct: %s | Reliability: %u%% | Pkts: %u | Last Seen: %lld ms ago",
                index,
                current_neighbor->address,
                current_neighbor->rssi,
                current_neighbor->is_direct ? "Yes" : "No",
                current_neighbor->reliability,
                current_neighbor->packet_count,
                age);
            current_neighbor = current_neighbor->next;
            index++;
        }
        LOG_INF("=== Total: %d neighbors ===", index);
    }
    k_mutex_unlock(&neighbor_list_mutex);

    index = 0; // Reset index for route list
    k_mutex_lock(&route_list_mutex, K_FOREVER);
    current_route = route_list;
    /* Display routing table */
    if (!current_route) {
        LOG_INF("Routing table is empty");
    } else {
        LOG_INF("=== Routing Table ===");
        while (current_route != NULL) {
            int64_t age = now - current_route->last_updated;
            bool is_stale = age > MESH_ROUTE_TIMEOUT_MS;

            LOG_INF("[%d] Dst: 0x%02X | Next Hop: 0x%02X | Hops: %d | Quality: %d | Age: %lld ms%s",
                index,
                current_route->dst,
                current_route->next_hop,
                current_route->hop_count,
                current_route->quality,
                age,
                is_stale ? " (STALE)" : "");
            current_route = current_route->next;
            index++;
        }
        LOG_INF("=== Total: %d routes ===", index);
    }
    k_mutex_unlock(&route_list_mutex);

    /* Schedule next display */
    k_work_schedule(&neighbors_list_work, K_SECONDS(MESH_NEIGHBOR_LIST_INTERVAL_SEC));
}

/**
 * @brief Send an acknowledgment for a received packet
 *
 * Sends an acknowledgment packet back to the source of a received packet.
 *
 * @param orig_src Original source of the packet to acknowledge
 * @param seq Sequence number of the packet to acknowledge
 */
void mesh_send_ack(uint32_t orig_src, uint8_t seq)
{
    hdr_t hdr = {0};
    mesh_t mesh = {0};
    uint8_t packet[MAX_PACKET_LEN] = {0};
    uint32_t next_hop;

    /* Find the next hop to reach the original source */
    next_hop = mesh_find_next_hop(orig_src);
    if (next_hop == 0) {
        LOG_ERR("Cannot send ACK to 0x%02X: no route available", orig_src);
        atomic_inc(&packets_dropped);
        return;
    }
    
    /* Update statistics */
    atomic_inc(&acks_sent);
    atomic_inc(&packets_sent);

    /* Prepare packet header */
    hdr.type = TYPE_MESH;
    hdr.size = sizeof(hdr_t) + sizeof(mesh_t);
    hdr.src = NODE_ADDRESS;
    hdr.dst = next_hop;
    hdr.seq_num = next_seq++;
    hdr.hop_count = 0;

    /* Prepare mesh payload */
    mesh.type = MESH_ACK;
    mesh.seq = seq;
    mesh.orig_src = NODE_ADDRESS;
    mesh.final_dst = orig_src;
    mesh.ttl = MESH_MAX_TTL;

    /* Assemble packet */
    memcpy(packet, &hdr, sizeof(hdr_t));
    memcpy(packet + sizeof(hdr_t), &mesh, sizeof(mesh_t));

    LOG_INF("Sending ACK for seq %d to 0x%02X via 0x%02X", 
            seq, orig_src, next_hop);
    
    /* Add a small delay to avoid collisions with other packets */
    k_msleep(50);
    
    lora_tx(packet, hdr.size);
}

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
int mesh_send_data(uint32_t dst, const uint8_t *data, size_t len)
{
    uint8_t packet[MAX_PACKET_LEN]; /* Use stack allocation for thread safety */
    hdr_t *hdr = (hdr_t *)packet;
    mesh_t *mesh = (mesh_t *)(packet + sizeof(hdr_t));
    uint32_t next_hop;
    uint8_t seq = next_seq++;
    pending_packet_t *pending;

    /* Input validation using standardized macros */
    VALIDATE_PTR(data);
    VALIDATE_ADDRESS(dst);
    
    if (len == 0) {
        LOG_ERR("Empty data provided to mesh_send_data");
        return MESH_ERR_INVALID_PARAM;
    }

    if (len > (MAX_PACKET_LEN - sizeof(hdr_t) - sizeof(mesh_t))) {
        LOG_ERR("Data too large for packet: %u bytes (max %u)", 
                len, MAX_PACKET_LEN - sizeof(hdr_t) - sizeof(mesh_t));
        return MESH_ERR_INVALID_PARAM;
    }

    /* Find the next hop to reach the destination */
    next_hop = mesh_find_next_hop(dst);
    if (next_hop == 0) {
        LOG_ERR("Cannot send to 0x%02X: no route available", dst);
        return -EHOSTUNREACH;
    }

    /* Clear the packet buffer */
    memset(packet, 0, sizeof(hdr_t) + sizeof(mesh_t));

    /* Prepare packet header */
    hdr->type = TYPE_MESH;
    hdr->size = sizeof(hdr_t) + sizeof(mesh_t) + len;
    hdr->src = NODE_ADDRESS;
    hdr->dst = next_hop;
    hdr->seq_num = seq;
    hdr->hop_count = 0;

    /* Prepare mesh payload */
    mesh->type = MESH_DATA;
    mesh->seq = seq;
    mesh->orig_src = NODE_ADDRESS;
    mesh->final_dst = dst;
    mesh->ttl = MESH_MAX_TTL;

    /* Copy data directly to packet */
    if (len > 0) {
        memcpy(packet + sizeof(hdr_t) + sizeof(mesh_t), data, len);
    }

    /* Allocate pending packet for retransmission if needed */
    if (k_mem_slab_alloc(&pending_packet_slab, (void **)&pending, K_NO_WAIT) == 0) {
        pending->dst = dst;
        pending->seq = seq;
        pending->retries = 0;
        pending->sent_time = k_uptime_get();
        pending->size = hdr->size;
        memcpy(pending->data, packet, hdr->size);
        
        /* Initialize retry work item */
        k_work_init_delayable(&pending->retry_work, mesh_retry_handler);
        
        k_mutex_lock(&pending_packet_list_mutex, K_FOREVER);
        /* Add to pending list */
        pending->next = pending_packet_list;
        pending_packet_list = pending;
        k_mutex_unlock(&pending_packet_list_mutex);
        
        /* Calculate timeout based on hop count and route quality */
        route_t *route = NULL;
        uint32_t timeout = MESH_ACK_TIMEOUT_BASE_MS;
        
        /* Find route to calculate appropriate timeout */
        k_mutex_lock(&route_list_mutex, K_FOREVER);
        route_t *current_route = route_list;
        while (current_route != NULL) {
            if (current_route->dst == dst) {
                route = current_route;
                break;
            }
            current_route = current_route->next;
        }
        k_mutex_unlock(&route_list_mutex);
        
        /* Adjust timeout based on hop count if route exists */
        if (route != NULL) {
            /* Add additional time for each hop */
            timeout += (route->hop_count - 1) * MESH_TIMEOUT_HOP_FACTOR_MS;
            
            /* Ensure timeout is within bounds */
            timeout = MIN(timeout, MESH_ACK_TIMEOUT_MAX_MS);
            timeout = MAX(timeout, MESH_ACK_TIMEOUT_MIN_MS);
            
            LOG_DBG("Adjusted ACK timeout for dst 0x%02X: %u ms (hop count: %d)", 
                    dst, timeout, route->hop_count);
        }
        
        /* Schedule retry if no ACK received */
        k_work_schedule_for_queue(&app_work_q, &pending->retry_work, K_MSEC(timeout));
    } else {
        LOG_WRN("No pending packet slots available, sending without reliability");
    }

    LOG_INF("Sending data packet (seq %d) to 0x%02X via 0x%02X, %u bytes", 
            seq, dst, next_hop, len);
    
    /* Update statistics */
    atomic_inc(&packets_sent);
    
    lora_tx(packet, hdr->size);
    
    return 0;
}

/**
 * @brief Handle retransmission of unacknowledged packets
 *
 * @param work Pointer to the work item
 */
static void mesh_retry_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    pending_packet_t *pending = CONTAINER_OF(dwork, pending_packet_t, retry_work);
    bool removed = false;
    
    k_mutex_lock(&pending_packet_list_mutex, K_FOREVER);
    // First, ensure the pending packet is still in the list (it might have been ACKed and removed)
    pending_packet_t *check_pending = pending_packet_list;
    bool still_pending = false;
    while(check_pending != NULL) {
        if (check_pending == pending) {
            still_pending = true;
            break;
        }
        check_pending = check_pending->next;
    }

    if (!still_pending) {
        k_mutex_unlock(&pending_packet_list_mutex);
        LOG_DBG("Packet seq %d to 0x%02X already ACKed or removed, skipping retry.", pending->seq, pending->dst);
        return;
    }

    if (pending->retries >= MESH_MAX_RETRANSMISSIONS) {
        LOG_ERR("Max retries reached for packet to 0x%02X, seq %d", 
                pending->dst, pending->seq);
        
        /* Update neighbor reliability metrics */
        k_mutex_unlock(&pending_packet_list_mutex); // Unlock before accessing neighbor_list
        k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
        neighbor_t *neighbor = neighbor_list;
        while (neighbor != NULL) {
            if (neighbor->address == pending->dst) {
                neighbor->failed_count++;
                
                /* Recalculate reliability score */
                if (neighbor->packet_count > 0) {
                    neighbor->reliability = 100 - (neighbor->failed_count * 100) / 
                                         (neighbor->packet_count + neighbor->failed_count);
                }
                
                LOG_WRN("Updated reliability for neighbor 0x%02X: %u%% (failures: %u, packets: %u)",
                        neighbor->address, neighbor->reliability, neighbor->failed_count, neighbor->packet_count);
                break;
            }
            neighbor = neighbor->next;
        }
        k_mutex_unlock(&neighbor_list_mutex);
        k_mutex_lock(&pending_packet_list_mutex, K_FOREVER); // Re-lock for pending list operations
        
        /* Remove from pending list */
        pending_packet_t **current = &pending_packet_list;
        while (*current != NULL) {
            if (*current == pending) {
                *current = pending->next;
                k_mem_slab_free(&pending_packet_slab, pending);
                removed = true;
                break;
            }
            current = &(*current)->next;
        }
        k_mutex_unlock(&pending_packet_list_mutex);
        return;
    }
    
    /* Increment retry count */
    pending->retries++;
    pending->sent_time = k_uptime_get();
    k_mutex_unlock(&pending_packet_list_mutex); // Unlock before calling mesh_find_next_hop
    
    /* Update statistics */
    atomic_inc(&retransmissions);
    
    /* Update next hop in case routes have changed */
    hdr_t *hdr = (hdr_t *)pending->data;
    mesh_t *mesh = (mesh_t *)(pending->data + sizeof(hdr_t));
    uint32_t next_hop = mesh_find_next_hop(mesh->final_dst); // This handles its own mutexes
    
    if (next_hop == 0) {
        LOG_ERR("Retry failed: no route to 0x%02X", mesh->final_dst);
        /* Schedule another retry in case a route becomes available */
        // No need to lock pending_packet_list_mutex here as we are just rescheduling
        k_work_schedule_for_queue(&app_work_q, &pending->retry_work, K_MSEC(MESH_RETRANSMISSION_INTERVAL_BASE_MS));
        return;
    }
    
    /* Update the next hop */
    // Lock is not needed for just updating hdr->dst as it's part of pending->data
    // which is specific to this pending item.
    hdr->dst = next_hop; 
    
    LOG_INF("Retrying packet to 0x%02X via 0x%02X (attempt %d/%d)", 
            mesh->final_dst, next_hop, pending->retries, MESH_MAX_RETRANSMISSIONS);
    
    /* Retransmit the packet */
    lora_tx(pending->data, pending->size); // lora_tx handles its own TX buffer mutex
    
    /* Schedule next retry */
    // No need to lock pending_packet_list_mutex here
    /* Calculate adaptive timeout based on hop count */
    uint32_t timeout = MESH_ACK_TIMEOUT_BASE_MS;
    
    /* Get the route to determine hop count */
    route_t *route = NULL;
    k_mutex_lock(&route_list_mutex, K_FOREVER);
    route_t *current_route = route_list;
    while (current_route != NULL) {
        if (current_route->dst == mesh->final_dst) {
            route = current_route;
            break;
        }
        current_route = current_route->next;
    }
    k_mutex_unlock(&route_list_mutex);
    
    /* Adjust timeout based on hop count if route exists */
    if (route != NULL) {
        /* Add additional time for each hop */
        timeout += (route->hop_count - 1) * MESH_TIMEOUT_HOP_FACTOR_MS;
        
        /* Ensure timeout is within bounds */
        timeout = MIN(timeout, MESH_ACK_TIMEOUT_MAX_MS);
        timeout = MAX(timeout, MESH_ACK_TIMEOUT_MIN_MS);
        
        LOG_DBG("Retry with adjusted timeout for dst 0x%02X: %u ms (hop count: %d)", 
                mesh->final_dst, timeout, route->hop_count);
    }
    
    /* Add randomization to avoid collisions (±20% of timeout) */
    int32_t jitter = (k_uptime_get() % (timeout / 5)) - (timeout / 10);
    timeout += jitter;
    
    LOG_DBG("Scheduling retry with timeout %u ms (including jitter %d ms)", 
            timeout, jitter);
    
    /* Schedule next retry with adaptive timeout */
    k_work_schedule(&pending->retry_work, K_MSEC(timeout));
}

/**
 * @brief Remove a pending packet from the list
 *
 * @param seq Sequence number of the packet to remove
 * @param dst Destination address of the packet
 * @return true if packet was found and removed, false otherwise
 */
static bool remove_pending_packet(uint8_t seq, uint32_t dst)
{
    pending_packet_t **current;
    bool removed = false;
    
    LOG_DBG("Looking for pending packet with seq %d to dst 0x%02X to remove", seq, dst);
    
    k_mutex_lock(&pending_packet_list_mutex, K_FOREVER);
    
    /* First, log all pending packets for debugging */
    pending_packet_t *debug_pending = pending_packet_list;
    int count = 0;
    while (debug_pending != NULL) {
        LOG_DBG("Pending packet %d: seq %d, dst 0x%02X, retries %d", 
                count++, debug_pending->seq, debug_pending->dst, debug_pending->retries);
        debug_pending = debug_pending->next;
    }
    
    current = &pending_packet_list;
    while (*current != NULL) {
        if ((*current)->seq == seq && (*current)->dst == dst) {
            pending_packet_t *to_remove = *current;
            *current = to_remove->next;
            
            /* Cancel any scheduled retries */
            k_work_cancel_delayable(&to_remove->retry_work);
            
            /* Free the memory */
            k_mem_slab_free(&pending_packet_slab, to_remove);
            
            LOG_INF("Removed acknowledged packet seq %d to 0x%02X", seq, dst);
            removed = true;
            break; 
        }
        current = &(*current)->next;
    }
    
    if (!removed) {
        LOG_WRN("Could not find pending packet with seq %d to dst 0x%02X", seq, dst);
    }
    
    k_mutex_unlock(&pending_packet_list_mutex);
    
    return removed;
}

/**
 * @brief Forward a packet to its next hop
 *
 * @param buf Pointer to the packet buffer
 * @return 0 on success, negative errno code on failure
 */
static int forward_packet(uint8_t *buf)
{
    hdr_t *hdr = (hdr_t *)buf;
    mesh_t *mesh = (mesh_t *)(buf + sizeof(hdr_t));
    uint32_t next_hop;
    
    /* Decrement TTL */
    if (mesh->ttl <= 1) {
        LOG_WRN("Dropping packet with expired TTL");
        atomic_inc(&packets_dropped);
        return -ETIMEDOUT;
    }
    mesh->ttl--;
    
    /* Increment hop count */
    hdr->hop_count++;
    
    /* Find next hop */
    next_hop = mesh_find_next_hop(mesh->final_dst);
    if (next_hop == 0) {
        LOG_ERR("Cannot forward packet: no route to 0x%02X", mesh->final_dst);
        atomic_inc(&packets_dropped);
        return -EHOSTUNREACH;
    }
    
    /* Update statistics */
    atomic_inc(&packets_forwarded);
    atomic_inc(&packets_sent);
    
    /* Update source and destination */
    hdr->src = NODE_ADDRESS;
    hdr->dst = next_hop;
    
    LOG_INF("Forwarding packet from 0x%02X to 0x%02X via 0x%02X (hop %d, ttl %d)", 
            mesh->orig_src, mesh->final_dst, next_hop, hdr->hop_count, mesh->ttl);
    
    /* Transmit the packet */
    lora_tx(buf, hdr->size);
    
    return 0;
}

/**
 * @brief Display network statistics
 *
 * Logs the current network statistics including packet counts,
 * reliability metrics, and performance indicators.
 *
 * @param work Pointer to the work item
 */
static void mesh_stats_display(struct k_work *work)
{
    int64_t now = k_uptime_get();
    int64_t time_diff_ms = now - stats.last_stats_time;
    
    /* Read atomic statistics for calculations */
    uint32_t current_packets_received = atomic_get(&packets_received);
    uint32_t current_packets_forwarded = atomic_get(&packets_forwarded);
    uint32_t current_packets_sent = atomic_get(&packets_sent);
    uint32_t current_retransmissions = atomic_get(&retransmissions);
    
    /* Calculate derived statistics */
    k_mutex_lock(&stats_mutex, K_FOREVER);
    if (time_diff_ms > 0 && stats.last_stats_time > 0) {
        /* Calculate packets per minute */
        double packets_per_min = (double)(current_packets_received + current_packets_forwarded) * 
                               (60000.0 / time_diff_ms);
        stats.packets_per_minute = (stats.packets_per_minute * 0.7) + (packets_per_min * 0.3);
        
        /* Calculate delivery success rate */
        if (current_packets_sent > 0) {
            double success_rate = 100.0 * (double)(current_packets_sent - current_retransmissions) / 
                                (double)current_packets_sent;
            stats.delivery_success_rate = (stats.delivery_success_rate * 0.7) + (success_rate * 0.3);
        }
    }
    k_mutex_unlock(&stats_mutex);
    
    /* Display statistics */
    LOG_INF("=== Network Statistics ===");
    
    /* Topology statistics */
    LOG_INF("Topology: %u neighbor changes, discovery interval: %u sec", 
            (uint32_t)atomic_get(&neighbor_changes), stats.current_discovery_interval);
    
    /* Packet statistics */
    LOG_INF("Packets: %u sent, %u received, %u forwarded, %u dropped", 
            current_packets_sent, current_packets_received, current_packets_forwarded, (uint32_t)atomic_get(&packets_dropped));
    
    /* Reliability statistics */
    LOG_INF("Reliability: %u ACKs sent, %u ACKs received, %u retransmissions", 
            (uint32_t)atomic_get(&acks_sent), (uint32_t)atomic_get(&acks_received), current_retransmissions);
    
    /* Performance statistics */
    LOG_INF("Performance: %.1f packets/min, %.1f%% delivery success rate", 
            stats.packets_per_minute, stats.delivery_success_rate);
    
    /* Update timestamp for next calculation */
    stats.last_stats_time = now;
    
    /* Schedule next statistics display */
    k_work_schedule(&stats_work, K_SECONDS(STATS_DISPLAY_INTERVAL_SEC));
}

/**
 * @brief Process a received mesh packet
 *
 * Handles different types of mesh packets (discovery, reply, data, etc.)
 * and updates the neighbor list accordingly. Implements multi-hop routing
 * for packets that need to be forwarded.
 *
 * @param buf Pointer to the packet buffer (including header)
 */
void mesh_process(uint8_t *buf)
{
    if (buf == NULL) {
        LOG_ERR("NULL buffer provided to mesh_process");
        return;
    }

    hdr_t *hdr = (hdr_t *)buf;
    mesh_t *mesh = (mesh_t *)(buf + sizeof(hdr_t));

    /* Validate source address */
    if (hdr->src == 0) {
        LOG_ERR("Invalid source address in mesh packet");
        return;
    }

    /* Ignore our own packets that might be relayed back */
    if (hdr->src == NODE_ADDRESS) {
        LOG_DBG("Ignoring our own mesh packet");
        return;
    }

    /* Update statistics */
    atomic_inc(&packets_received);

/* Update neighbor information for the direct sender */
add_or_update_neighbor(hdr->src, hdr->rssi);

/* Update route quality for the original source if this is a multi-hop packet */
if (mesh->type == MESH_DATA || mesh->type == MESH_ACK) {
    update_route_quality(mesh->orig_src, hdr->rssi);
}

    /* Process based on mesh packet type */
    switch (mesh->type) {
        case MESH_DISCOVER:
            LOG_INF("Received discovery request from 0x%02X", hdr->src);
            mesh_reply(hdr->src);
            break;
            
        case MESH_REPLY:
            LOG_INF("Received discovery reply from 0x%02X (RSSI: %d)", 
                    hdr->src, hdr->rssi);
            break;
            
        case MESH_DATA:
            /* Check if we are the final destination */
            if (mesh->final_dst == NODE_ADDRESS) {
                LOG_INF("Received data packet from 0x%02X (seq %d, hops %d)", 
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
            } else {
                /* Forward the packet */
                LOG_INF("Forwarding data packet from 0x%02X to 0x%02X", 
                        mesh->orig_src, mesh->final_dst);
                forward_packet(buf);
            }
            break;
            
        case MESH_ACK:
            /* Check if we are the final destination */
            if (mesh->final_dst == NODE_ADDRESS) {
                LOG_INF("Received ACK from 0x%02X for seq %d", 
                        mesh->orig_src, mesh->seq);
                
                /* Update statistics */
                atomic_inc(&acks_received);
                
                /* Check if this is an ACK for a broadcast packet */
                if (process_broadcast_ack(mesh->orig_src, mesh->seq)) {
                    LOG_INF("Processed ACK for broadcast packet seq %d from 0x%02X", 
                            mesh->seq, mesh->orig_src);
                } 
                /* Otherwise, check if it's for a regular packet */
                else if (remove_pending_packet(mesh->seq, mesh->orig_src)) {
                    LOG_INF("Successfully processed ACK for seq %d from 0x%02X", 
                            mesh->seq, mesh->orig_src);
                } else {
                    LOG_WRN("ACK received for unknown packet: seq %d from 0x%02X", 
                            mesh->seq, mesh->orig_src);
                }
            } else {
                /* Forward the ACK */
                LOG_INF("Forwarding ACK from 0x%02X to 0x%02X", 
                        mesh->orig_src, mesh->final_dst);
                forward_packet(buf);
            }
            break;
            
        case MESH_BROADCAST:
            LOG_INF("Received broadcast packet from 0x%02X (seq %d, hops %d)", 
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
            break;
            
        case MESH_ROUTE_REQ:
            /* Not implemented yet */
            LOG_WRN("Route request not implemented yet");
            break;
            
        case MESH_ROUTE_RESP:
            /* Not implemented yet */
            LOG_WRN("Route response not implemented yet");
            break;
            
        default:
            LOG_ERR("Invalid mesh packet type: %d", mesh->type);
            break;
    }
}

/**
 * @brief Get the head of the neighbor list
 * 
 * This function is used by other modules to access the neighbor list
 * for iteration. It returns the head of the neighbor list with proper
 * mutex protection.
 * 
 * @return Pointer to the first neighbor, or NULL if the list is empty
 */
void *get_neighbor_list(void)
{
    void *result = NULL;
    
    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    result = neighbor_list;
    k_mutex_unlock(&neighbor_list_mutex);
    
    return result;
}

/**
 * @brief Get the next neighbor in the list
 * 
 * This function is used to iterate through the neighbor list.
 * It returns the next neighbor in the list with proper mutex protection.
 * 
 * @param current Pointer to the current neighbor
 * @return Pointer to the next neighbor, or NULL if at the end of the list
 */
void *get_next_neighbor(void *current)
{
    if (current == NULL) {
        return NULL;
    }
    
    void *result = NULL;
    neighbor_t *neighbor = (neighbor_t *)current;
    
    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    result = neighbor->next;
    k_mutex_unlock(&neighbor_list_mutex);
    
    return result;
}

/**
 * @brief Get the address of a neighbor
 * 
 * This function returns the address of a neighbor with proper mutex protection.
 * 
 * @param neighbor Pointer to the neighbor
 * @return Address of the neighbor, or 0 if the pointer is invalid
 */
uint32_t get_neighbor_address(void *neighbor)
{
    if (neighbor == NULL) {
        return 0;
    }
    
    uint32_t result = 0;
    
    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    result = ((neighbor_t *)neighbor)->address;
    k_mutex_unlock(&neighbor_list_mutex);
    
    return result;
}

/**
 * @brief Get the reliability score of a neighbor
 * 
 * This function returns the reliability score of a neighbor with proper mutex protection.
 * The reliability score is a value from 0-100 where higher is better.
 * 
 * @param neighbor Pointer to the neighbor
 * @return Reliability score (0-100), or 0 if the pointer is invalid
 */
uint8_t get_neighbor_reliability(void *neighbor)
{
    if (neighbor == NULL) {
        return 0;
    }
    
    uint8_t result = 0;
    
    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    result = ((neighbor_t *)neighbor)->reliability;
    k_mutex_unlock(&neighbor_list_mutex);
    
    return result;
}

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
int mesh_send_broadcast(const uint8_t *data, size_t len)
{
    uint8_t packet[MAX_PACKET_LEN]; /* Use stack allocation for thread safety */
    hdr_t *hdr = (hdr_t *)packet;
    mesh_t *mesh = (mesh_t *)(packet + sizeof(hdr_t));
    uint8_t seq = next_seq++;
    broadcast_ack_tracker_t *tracker;
    int neighbor_count = 0;
    
    /* Input validation using standardized macros */
    VALIDATE_PTR(data);
    
    if (len == 0) {
        LOG_ERR("Empty data provided to mesh_send_broadcast");
        return MESH_ERR_INVALID_PARAM;
    }

    if (len > (MAX_PACKET_LEN - sizeof(hdr_t) - sizeof(mesh_t))) {
        LOG_ERR("Data too large for packet: %u bytes (max %u)", 
                len, MAX_PACKET_LEN - sizeof(hdr_t) - sizeof(mesh_t));
        return MESH_ERR_INVALID_PARAM;
    }
    
    /* Count neighbors and check if we have any */
    k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
    neighbor_t *neighbor = neighbor_list;
    while (neighbor != NULL) {
        if (neighbor->address != 0 && neighbor->address != NODE_ADDRESS && 
            neighbor->is_direct && neighbor->reliability >= 30) {
            neighbor_count++;
        }
        neighbor = neighbor->next;
    }
    k_mutex_unlock(&neighbor_list_mutex);
    
    if (neighbor_count == 0) {
        LOG_WRN("No reliable neighbors found for broadcast");
        return -EHOSTUNREACH;
    }
    
    /* Clear the packet buffer */
    memset(packet, 0, sizeof(hdr_t) + sizeof(mesh_t));

    /* Prepare packet header */
    hdr->type = TYPE_MESH;
    hdr->size = sizeof(hdr_t) + sizeof(mesh_t) + len;
    hdr->src = NODE_ADDRESS;
    hdr->dst = 0; /* Broadcast */
    hdr->seq_num = seq;
    hdr->hop_count = 0;

    /* Prepare mesh payload */
    mesh->type = MESH_BROADCAST;
    mesh->seq = seq;
    mesh->orig_src = NODE_ADDRESS;
    mesh->final_dst = 0; /* Broadcast */
    mesh->ttl = MESH_MAX_TTL;

    /* Copy data directly to packet */
    if (len > 0) {
        memcpy(packet + sizeof(hdr_t) + sizeof(mesh_t), data, len);
    }
    
    /* Allocate broadcast tracker for ACK tracking */
    if (k_mem_slab_alloc(&broadcast_tracker_slab, (void **)&tracker, K_NO_WAIT) == 0) {
        tracker->seq = seq;
        tracker->retries = 0;
        tracker->sent_time = k_uptime_get();
        tracker->size = hdr->size;
        memcpy(tracker->data, packet, hdr->size);
        
        /* Initialize retry work item */
        k_work_init_delayable(&tracker->retry_work, mesh_broadcast_retry_handler);
        
        /* Set up neighbor tracking */
        tracker->expected_acks = 0;
        tracker->received_acks = 0;
        
        /* Add neighbors to track */
        k_mutex_lock(&neighbor_list_mutex, K_FOREVER);
        neighbor = neighbor_list;
        while (neighbor != NULL && tracker->expected_acks < MESH_MAX_NEIGHBORS) {
            if (neighbor->address != 0 && neighbor->address != NODE_ADDRESS && 
                neighbor->is_direct && neighbor->reliability >= 30) {
                tracker->neighbor_addrs[tracker->expected_acks] = neighbor->address;
                tracker->neighbor_acked[tracker->expected_acks] = false;
                tracker->expected_acks++;
            }
            neighbor = neighbor->next;
        }
        k_mutex_unlock(&neighbor_list_mutex);
        
        /* Add to tracker list */
        k_mutex_lock(&broadcast_tracker_mutex, K_FOREVER);
        tracker->next = broadcast_tracker_list;
        broadcast_tracker_list = tracker;
        k_mutex_unlock(&broadcast_tracker_mutex);
        
        /* Schedule retry if not all ACKs received */
        uint32_t timeout = MESH_ACK_TIMEOUT_BASE_MS;
        
        /* Add randomization to avoid collisions (±20% of timeout) */
        int32_t jitter = (k_uptime_get() % (timeout / 5)) - (timeout / 10);
        timeout += jitter;
        
        LOG_DBG("Scheduling broadcast retry with timeout %u ms (including jitter %d ms)", 
                timeout, jitter);
        
        k_work_schedule_for_queue(&app_work_q, &tracker->retry_work, K_MSEC(timeout));
    } else {
        LOG_WRN("No broadcast tracker slots available, sending without reliability");
    }
    
    LOG_INF("Broadcasting data packet (seq %d) to all neighbors, %u bytes, expecting %d ACKs", 
            seq, len, neighbor_count);
    
    /* Update statistics */
    atomic_inc(&packets_sent);
    
    /* Send the packet */
    lora_tx(packet, hdr->size);
    
    return 0;
}

/**
 * @brief Process an acknowledgment for a broadcast packet
 * 
 * Updates the broadcast ACK tracker when an ACK is received for a broadcast packet.
 * 
 * @param src Source address of the ACK
 * @param seq Sequence number of the broadcast packet being acknowledged
 * @return true if the ACK was processed, false if no matching tracker was found
 */
static bool process_broadcast_ack(uint32_t src, uint8_t seq)
{
    broadcast_ack_tracker_t *tracker;
    bool processed = false;
    
    k_mutex_lock(&broadcast_tracker_mutex, K_FOREVER);
    
    tracker = broadcast_tracker_list;
    while (tracker != NULL) {
        if (tracker->seq == seq) {
            /* Found the tracker for this sequence number */
            for (int i = 0; i < tracker->expected_acks; i++) {
                if (tracker->neighbor_addrs[i] == src && !tracker->neighbor_acked[i]) {
                    /* Mark this neighbor as having ACKed */
                    tracker->neighbor_acked[i] = true;
                    tracker->received_acks++;
                    
                    LOG_INF("Received ACK for broadcast packet seq %d from neighbor 0x%02X (%d/%d ACKs received)", 
                            seq, src, tracker->received_acks, tracker->expected_acks);
                    
                    processed = true;
                    break;
                }
            }
            
            /* If all neighbors have ACKed, remove the tracker */
            if (tracker->received_acks >= tracker->expected_acks) {
                LOG_INF("All %d neighbors have acknowledged broadcast packet seq %d", 
                        tracker->expected_acks, seq);
                
                /* Cancel the retry work */
                k_work_cancel_delayable(&tracker->retry_work);
                
                /* Remove from tracker list - need to restart the loop */
                broadcast_ack_tracker_t **current = &broadcast_tracker_list;
                while (*current != NULL) {
                    if (*current == tracker) {
                        *current = tracker->next;
                        k_mem_slab_free(&broadcast_tracker_slab, tracker);
                        break;
                    }
                    current = &(*current)->next;
                }
            }
            
            break;
        }
        
        tracker = tracker->next;
    }
    
    k_mutex_unlock(&broadcast_tracker_mutex);
    
    return processed;
}

/**
 * @brief Handler for deferred route creation
 *
 * Creates a route for a newly added neighbor without holding the neighbor mutex.
 *
 * @param work Pointer to the work item
 */
static void add_route_work_handler(struct k_work *work)
{
    add_or_update_route(route_work_data.address, route_work_data.address, 1, route_work_data.rssi);
}

/**
 * @brief Initialize the mesh network module
 *
 * Sets up the necessary work items for periodic mesh operations
 * such as neighbor discovery, maintenance, and pruning.
 */
void mesh_init(void)
{
    /* Initialize work items */
    k_work_init_delayable(&discover_work, mesh_discover);
    k_work_init_delayable(&neighbors_list_work, mesh_neighbor_list);
    k_work_init_delayable(&prune_work, prune_work_handler);
    k_work_init_delayable(&stats_work, mesh_stats_display);
    k_work_init_delayable(&route_prune_work, route_prune_work_handler);
    k_work_init(&add_route_work, add_route_work_handler);

    /* Initialize network statistics */
    stats.last_topology_change = 0;
    stats.current_discovery_interval = MESH_DISCOVERY_INTERVAL_BASE_SEC;
    stats.last_stats_time = k_uptime_get();
    stats.packets_per_minute = 0.0;
    stats.delivery_success_rate = 100.0;

    /* Schedule initial work with staggered delays */
    k_work_schedule_for_queue(&app_work_q, &discover_work, K_SECONDS(MESH_INITIAL_DISCOVERY_DELAY_SEC));
    k_work_schedule_for_queue(&app_work_q, &neighbors_list_work, K_SECONDS(MESH_NEIGHBOR_LIST_INTERVAL_SEC));
    k_work_schedule_for_queue(&app_work_q, &prune_work, K_SECONDS(MESH_PRUNE_INTERVAL_SEC));
    k_work_schedule_for_queue(&app_work_q, &stats_work, K_SECONDS(STATS_DISPLAY_INTERVAL_SEC));
    k_work_schedule_for_queue(&app_work_q, &route_prune_work, K_SECONDS(MESH_ROUTE_PRUNE_INTERVAL_SEC));

    LOG_INF("Mesh network initialized with adaptive discovery (base interval: %d sec)",
            MESH_DISCOVERY_INTERVAL_BASE_SEC);
}
