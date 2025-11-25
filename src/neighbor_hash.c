/**
 * @file neighbor_hash.c
 * @brief Hash table implementation for efficient neighbor lookup
 *
 * This module provides O(1) average-case neighbor lookup performance
 * to replace the O(n) linear search as networks grow larger.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <string.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(neighbor_hash);

#include <neighbor_hash.h>
#include <config.h>

/**
 * @brief Neighbor hash table bucket structure
 */
typedef struct neighbor_hash_bucket {
    neighbor_t *head;          /**< Head of the chain for this bucket */
    uint32_t count;            /**< Number of neighbors in this bucket */
} neighbor_hash_bucket_t;

/* Hash table for efficient neighbor lookups */
static neighbor_hash_bucket_t neighbor_hash_table[NEIGHBOR_HASH_SIZE];

/* Mutex for hash table protection */
K_MUTEX_DEFINE(neighbor_hash_mutex);

/* Hash table statistics */
static struct {
    uint32_t total_neighbors;
    uint32_t total_lookups;
    uint32_t hash_collisions;
    uint32_t max_chain_length;
} hash_stats = {0};

/**
 * @brief Initialize the neighbor hash table
 *
 * Sets up the hash table for efficient neighbor lookups.
 * Must be called before any hash table operations.
 */
void neighbor_hash_init(void)
{
    k_mutex_lock(&neighbor_hash_mutex, K_FOREVER);
    
    /* Initialize all hash buckets */
    for (int i = 0; i < NEIGHBOR_HASH_SIZE; i++) {
        neighbor_hash_table[i].head = NULL;
        neighbor_hash_table[i].count = 0;
    }
    
    /* Reset statistics */
    memset(&hash_stats, 0, sizeof(hash_stats));
    
    k_mutex_unlock(&neighbor_hash_mutex);
    
    LOG_INF("Neighbor hash table initialized with %d buckets", NEIGHBOR_HASH_SIZE);
}

/**
 * @brief Add a neighbor to the hash table
 *
 * Adds a neighbor to both the linked list and hash table
 * for efficient access.
 *
 * @param neighbor Pointer to the neighbor to add
 * @return 0 on success, negative error code on failure
 */
int neighbor_hash_add(neighbor_t *neighbor)
{
    if (neighbor == NULL || neighbor->address == 0) {
        return MESH_ERR_INVALID_PARAM;
    }
    
    uint8_t hash = neighbor_hash_func(neighbor->address);
    
    k_mutex_lock(&neighbor_hash_mutex, K_FOREVER);
    
    /* Check if neighbor already exists in this bucket */
    neighbor_t *current = neighbor_hash_table[hash].head;
    while (current != NULL) {
        if (current->address == neighbor->address) {
            LOG_WRN("Neighbor 0x%02X already exists in hash table", neighbor->address);
            k_mutex_unlock(&neighbor_hash_mutex);
            return MESH_ERR_INVALID_STATE;
        }
        current = current->hash_next;
    }
    
    /* Add to head of the hash bucket chain */
    neighbor->hash_next = neighbor_hash_table[hash].head;
    neighbor_hash_table[hash].head = neighbor;
    neighbor_hash_table[hash].count++;
    
    /* Update statistics */
    hash_stats.total_neighbors++;
    if (neighbor_hash_table[hash].count > 1) {
        hash_stats.hash_collisions++;
    }
    if (neighbor_hash_table[hash].count > hash_stats.max_chain_length) {
        hash_stats.max_chain_length = neighbor_hash_table[hash].count;
    }
    
    k_mutex_unlock(&neighbor_hash_mutex);
    
    LOG_DBG("Added neighbor 0x%02X to hash bucket %d (chain length: %u)", 
            neighbor->address, hash, neighbor_hash_table[hash].count);
    
    return 0;
}

/**
 * @brief Remove a neighbor from the hash table
 *
 * Removes a neighbor from both the linked list and hash table.
 *
 * @param address Address of the neighbor to remove
 * @return Pointer to removed neighbor, or NULL if not found
 */
neighbor_t *neighbor_hash_remove(uint8_t address)
{
    if (address == 0) {
        return NULL;
    }
    
    uint8_t hash = neighbor_hash_func(address);
    neighbor_t *removed = NULL;
    
    k_mutex_lock(&neighbor_hash_mutex, K_FOREVER);
    
    /* Find and remove from hash bucket chain */
    neighbor_t **current = &neighbor_hash_table[hash].head;
    while (*current != NULL) {
        if ((*current)->address == address) {
            removed = *current;
            *current = removed->hash_next;
            neighbor_hash_table[hash].count--;
            hash_stats.total_neighbors--;
            
            /* Clear the hash_next pointer */
            removed->hash_next = NULL;
            
            LOG_DBG("Removed neighbor 0x%02X from hash bucket %d (chain length: %u)", 
                    address, hash, neighbor_hash_table[hash].count);
            break;
        }
        current = &(*current)->hash_next;
    }
    
    k_mutex_unlock(&neighbor_hash_mutex);
    
    if (removed == NULL) {
        LOG_WRN("Neighbor 0x%02X not found in hash table", address);
    }
    
    return removed;
}

/**
 * @brief Find a neighbor in the hash table
 *
 * Efficiently finds a neighbor using hash table lookup.
 *
 * @param address Address of the neighbor to find
 * @return Pointer to neighbor, or NULL if not found
 */
neighbor_t *neighbor_hash_find(uint8_t address)
{
    if (address == 0) {
        return NULL;
    }
    
    uint8_t hash = neighbor_hash_func(address);
    neighbor_t *found = NULL;
    
    k_mutex_lock(&neighbor_hash_mutex, K_FOREVER);
    
    hash_stats.total_lookups++;
    
    /* Search the hash bucket chain */
    neighbor_t *current = neighbor_hash_table[hash].head;
    while (current != NULL) {
        if (current->address == address) {
            found = current;
            break;
        }
        current = current->hash_next;
    }
    
    k_mutex_unlock(&neighbor_hash_mutex);
    
    LOG_DBG("Lookup for neighbor 0x%02X in bucket %d: %s", 
            address, hash, found ? "found" : "not found");
    
    return found;
}

/**
 * @brief Update a neighbor's hash table entry
 *
 * Updates the hash table when neighbor information changes.
 * Currently a no-op since address doesn't change, but kept
 * for future extensibility.
 *
 * @param neighbor Pointer to the updated neighbor
 * @return 0 on success, negative error code on failure
 */
int neighbor_hash_update(neighbor_t *neighbor)
{
    if (neighbor == NULL || neighbor->address == 0) {
        return MESH_ERR_INVALID_PARAM;
    }
    
    /* Currently no update needed since address doesn't change */
    /* This function is kept for future extensibility */
    return 0;
}

/**
 * @brief Clear all entries from the hash table
 *
 * Removes all neighbors from the hash table and linked list.
 * Used for cleanup and testing purposes.
 */
void neighbor_hash_clear(void)
{
    k_mutex_lock(&neighbor_hash_mutex, K_FOREVER);
    
    /* Clear all hash buckets */
    for (int i = 0; i < NEIGHBOR_HASH_SIZE; i++) {
        neighbor_hash_table[i].head = NULL;
        neighbor_hash_table[i].count = 0;
    }
    
    /* Reset statistics */
    memset(&hash_stats, 0, sizeof(hash_stats));
    
    k_mutex_unlock(&neighbor_hash_mutex);
    
    LOG_INF("Neighbor hash table cleared");
}

/**
 * @brief Get hash table statistics
 *
 * Returns information about hash table performance for debugging.
 *
 * @param total_neighbors Total number of neighbors in table
 * @param max_chain_length Longest chain in any bucket
 * @param empty_buckets Number of empty hash buckets
 */
void neighbor_hash_stats(uint32_t *total_neighbors, uint32_t *max_chain_length, uint32_t *empty_buckets)
{
    if (total_neighbors == NULL || max_chain_length == NULL || empty_buckets == NULL) {
        return;
    }
    
    k_mutex_lock(&neighbor_hash_mutex, K_FOREVER);
    
    *total_neighbors = hash_stats.total_neighbors;
    *max_chain_length = hash_stats.max_chain_length;
    
    /* Count empty buckets */
    uint32_t empty = 0;
    for (int i = 0; i < NEIGHBOR_HASH_SIZE; i++) {
        if (neighbor_hash_table[i].count == 0) {
            empty++;
        }
    }
    *empty_buckets = empty;
    
    k_mutex_unlock(&neighbor_hash_mutex);
    
    LOG_INF("Hash table stats: %u neighbors, max chain %u, %u empty buckets, %u lookups, %u collisions",
            hash_stats.total_neighbors, hash_stats.max_chain_length, empty,
            hash_stats.total_lookups, hash_stats.hash_collisions);
}
