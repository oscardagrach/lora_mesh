/**
 * @file neighbor_hash.h
 * @brief Hash table implementation for efficient neighbor lookup
 *
 * This module provides O(1) average-case neighbor lookup performance
 * to replace the O(n) linear search as networks grow larger.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef NEIGHBOR_HASH_H
#define NEIGHBOR_HASH_H

#include <stdint.h>
#include <stdbool.h>
#include <config.h>

/**
 * @brief Hash table size for neighbor lookups
 * Should be a power of 2 for efficient modulo operations
 */
#define NEIGHBOR_HASH_SIZE 32

/**
 * @brief Forward declaration of neighbor structure
 */
typedef struct neighbor neighbor_t;

/**
 * @brief Initialize the neighbor hash table
 *
 * Sets up the hash table for efficient neighbor lookups.
 * Must be called before any hash table operations.
 */
void neighbor_hash_init(void);

/**
 * @brief Add a neighbor to the hash table
 *
 * Adds a neighbor to both the linked list and hash table
 * for efficient access.
 *
 * @param neighbor Pointer to the neighbor to add
 * @return 0 on success, negative error code on failure
 */
int neighbor_hash_add(neighbor_t *neighbor);

/**
 * @brief Remove a neighbor from the hash table
 *
 * Removes a neighbor from both the linked list and hash table.
 *
 * @param address Address of the neighbor to remove
 * @return Pointer to removed neighbor, or NULL if not found
 */
neighbor_t *neighbor_hash_remove(uint8_t address);

/**
 * @brief Find a neighbor in the hash table
 *
 * Efficiently finds a neighbor using hash table lookup.
 *
 * @param address Address of the neighbor to find
 * @return Pointer to neighbor, or NULL if not found
 */
neighbor_t *neighbor_hash_find(uint8_t address);

/**
 * @brief Update a neighbor's hash table entry
 *
 * Updates the hash table when neighbor information changes.
 * Handles cases where the neighbor might need to be moved
 * to a different hash bucket.
 *
 * @param neighbor Pointer to the updated neighbor
 * @return 0 on success, negative error code on failure
 */
int neighbor_hash_update(neighbor_t *neighbor);

/**
 * @brief Clear all entries from the hash table
 *
 * Removes all neighbors from the hash table and linked list.
 * Used for cleanup and testing purposes.
 */
void neighbor_hash_clear(void);

/**
 * @brief Get hash table statistics
 *
 * Returns information about hash table performance for debugging.
 *
 * @param total_neighbors Total number of neighbors in table
 * @param max_chain_length Longest chain in any bucket
 * @param empty_buckets Number of empty hash buckets
 */
void neighbor_hash_stats(uint32_t *total_neighbors, uint32_t *max_chain_length, uint32_t *empty_buckets);

/**
 * @brief Calculate hash value for an address
 *
 * Simple hash function optimized for uniform distribution
 * of neighbor addresses.
 *
 * @param address Node address to hash
 * @return Hash value (0 to NEIGHBOR_HASH_SIZE-1)
 */
static inline uint8_t neighbor_hash_func(uint8_t address) {
    /* Simple hash using multiplication and bit shifting for better distribution */
    return (address * 31) % NEIGHBOR_HASH_SIZE;
}

#endif /* NEIGHBOR_HASH_H */
