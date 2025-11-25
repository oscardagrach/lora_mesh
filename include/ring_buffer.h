/**
 * @file ring_buffer.h
 * @brief Thread-safe ring buffer implementation for packet queuing
 *
 * This module provides separate ring buffers for RX and TX operations,
 * with thread-safe access through mutex protection.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>

/**
 * @brief Read data from the RX ring buffer
 *
 * Thread-safe function to read data from the RX ring buffer.
 * The data is removed from the buffer after reading.
 *
 * @param data Pointer to the buffer where data will be stored
 * @param len Maximum number of bytes to read
 * @return Number of bytes actually read, or negative error code
 */
uint32_t rb_read_rx(uint8_t *data, size_t len);

/**
 * @brief Write data to the RX ring buffer
 *
 * Thread-safe function to write data to the RX ring buffer.
 *
 * @param data Pointer to the data to write
 * @param len Number of bytes to write
 * @return Number of bytes actually written
 */
int rb_write_rx(const uint8_t *data, size_t len);

/**
 * @brief Peek data from the RX ring buffer without removing it
 *
 * Thread-safe function to read data from the RX ring buffer
 * without removing it from the buffer.
 *
 * @param data Pointer to the buffer where data will be stored
 * @param len Maximum number of bytes to peek
 * @return Number of bytes actually peeked, or negative error code
 */
uint32_t rb_peek_rx(uint8_t *data, size_t len);

/**
 * @brief Get the number of bytes used in the RX ring buffer
 *
 * Thread-safe function to get the number of bytes currently
 * stored in the RX ring buffer.
 *
 * @return Number of bytes used
 */
size_t rb_size_get_rx(void);

/**
 * @brief Get the amount of free space in the RX ring buffer
 *
 * Thread-safe function to get the number of bytes that can
 * still be written to the RX ring buffer.
 *
 * @return Number of bytes available
 */
size_t rb_space_get_rx(void);

/**
 * @brief Read data from the TX ring buffer
 *
 * Thread-safe function to read data from the TX ring buffer.
 * The data is removed from the buffer after reading.
 *
 * @param data Pointer to the buffer where data will be stored
 * @param len Maximum number of bytes to read
 * @return Number of bytes actually read, or negative error code
 */
uint32_t rb_read_tx(uint8_t *data, size_t len);

/**
 * @brief Write data to the TX ring buffer
 *
 * Thread-safe function to write data to the TX ring buffer.
 *
 * @param data Pointer to the data to write
 * @param len Number of bytes to write
 * @return Number of bytes actually written
 */
int rb_write_tx(const uint8_t *data, size_t len);

/**
 * @brief Peek data from the TX ring buffer without removing it
 *
 * Thread-safe function to read data from the TX ring buffer
 * without removing it from the buffer.
 *
 * @param data Pointer to the buffer where data will be stored
 * @param len Maximum number of bytes to peek
 * @return Number of bytes actually peeked, or negative error code
 */
uint32_t rb_peek_tx(uint8_t *data, size_t len);

/**
 * @brief Get the number of bytes used in the TX ring buffer
 *
 * Thread-safe function to get the number of bytes currently
 * stored in the TX ring buffer.
 *
 * @return Number of bytes used
 */
size_t rb_size_get_tx(void);

/**
 * @brief Get the amount of free space in the TX ring buffer
 *
 * Thread-safe function to get the number of bytes that can
 * still be written to the TX ring buffer.
 *
 * @return Number of bytes available
 */
size_t rb_space_get_tx(void);

#endif /* RING_BUFFER_H */
