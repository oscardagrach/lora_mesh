/**
 * @file ring_buffer.c
 * @brief Thread-safe ring buffer implementation for packet queuing
 *
 * This module provides separate ring buffers for RX and TX operations,
 * with thread-safe access through mutex protection.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <ring_buffer.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ring_buffer);

/* Buffer sizes defined in packet.h for consistency */
#define RX_BUF_LEN 2048
#define TX_BUF_LEN 2048

/* Mutexes for thread-safe access to ring buffers */
K_MUTEX_DEFINE(rx_mutex);
K_MUTEX_DEFINE(tx_mutex);

/* Ring buffer declarations */
RING_BUF_DECLARE(rx_buf, RX_BUF_LEN);
RING_BUF_DECLARE(tx_buf, TX_BUF_LEN);

/**
 * @brief Generic ring buffer operation with mutex protection
 *
 * This helper function performs a ring buffer operation with proper
 * mutex protection to ensure thread safety.
 *
 * @param mutex Pointer to the mutex to lock
 * @param buf Pointer to the ring buffer
 * @param op Function pointer to the ring buffer operation to perform
 * @param data Pointer to the data buffer
 * @param len Length of data to operate on
 * @return Result of the ring buffer operation
 */
static inline uint32_t rb_op(struct k_mutex *mutex, struct ring_buf *buf,
                             uint32_t (*op)(struct ring_buf*, uint8_t*, uint32_t),
                             uint8_t *data, size_t len)
{
    uint32_t ret;
    
    if (data == NULL) {
        return -EINVAL;
    }
    
    k_mutex_lock(mutex, K_FOREVER);
    ret = op(buf, data, len);
    k_mutex_unlock(mutex);
    
    return ret;
}

/* ========================================================================== */
/* RX BUFFER FUNCTIONS                                                        */
/* ========================================================================== */

uint32_t rb_read_rx(uint8_t *data, size_t len)
{
    if (data == NULL) {
        LOG_ERR("NULL data pointer provided to rb_read_rx");
        return -EINVAL;
    }
    
    return rb_op(&rx_mutex, &rx_buf, ring_buf_get, data, len);
}

int rb_write_rx(const uint8_t *data, size_t len)
{
    int ret;
    
    if (data == NULL) {
        LOG_ERR("NULL data pointer provided to rb_write_rx");
        return -EINVAL;
    }
    
    k_mutex_lock(&rx_mutex, K_FOREVER);
    ret = ring_buf_put(&rx_buf, data, len);
    
    /* Check if we couldn't write all data */
    if (ret < len) {
        LOG_WRN("RX buffer overflow: requested %u bytes, wrote %d bytes", len, ret);
    }
    
    k_mutex_unlock(&rx_mutex);
    return ret;
}

uint32_t rb_peek_rx(uint8_t *data, size_t len)
{
    if (data == NULL) {
        LOG_ERR("NULL data pointer provided to rb_peek_rx");
        return -EINVAL;
    }
    
    return rb_op(&rx_mutex, &rx_buf, ring_buf_peek, data, len);
}

size_t rb_size_get_rx(void)
{
    size_t used_space;
    
    k_mutex_lock(&rx_mutex, K_FOREVER);
    used_space = ring_buf_size_get(&rx_buf);
    k_mutex_unlock(&rx_mutex);
    
    return used_space;
}

size_t rb_space_get_rx(void)
{
    size_t available_space;
    
    k_mutex_lock(&rx_mutex, K_FOREVER);
    available_space = ring_buf_space_get(&rx_buf);
    k_mutex_unlock(&rx_mutex);
    
    return available_space;
}

/* ========================================================================== */
/* TX BUFFER FUNCTIONS                                                        */
/* ========================================================================== */

uint32_t rb_read_tx(uint8_t *data, size_t len)
{
    if (data == NULL) {
        LOG_ERR("NULL data pointer provided to rb_read_tx");
        return -EINVAL;
    }
    
    return rb_op(&tx_mutex, &tx_buf, ring_buf_get, data, len);
}

int rb_write_tx(const uint8_t *data, size_t len)
{
    int ret;
    
    if (data == NULL) {
        LOG_ERR("NULL data pointer provided to rb_write_tx");
        return -EINVAL;
    }
    
    k_mutex_lock(&tx_mutex, K_FOREVER);
    ret = ring_buf_put(&tx_buf, data, len);
    
    /* Check if we couldn't write all data */
    if (ret < len) {
        LOG_WRN("TX buffer overflow: requested %u bytes, wrote %d bytes", len, ret);
    }
    
    k_mutex_unlock(&tx_mutex);
    return ret;
}

uint32_t rb_peek_tx(uint8_t *data, size_t len)
{
    if (data == NULL) {
        LOG_ERR("NULL data pointer provided to rb_peek_tx");
        return -EINVAL;
    }
    
    return rb_op(&tx_mutex, &tx_buf, ring_buf_peek, data, len);
}

size_t rb_size_get_tx(void)
{
    size_t used_space;
    
    k_mutex_lock(&tx_mutex, K_FOREVER);
    used_space = ring_buf_size_get(&tx_buf);
    k_mutex_unlock(&tx_mutex);
    
    return used_space;
}

size_t rb_space_get_tx(void)
{
    size_t available_space;
    
    k_mutex_lock(&tx_mutex, K_FOREVER);
    available_space = ring_buf_space_get(&tx_buf);
    k_mutex_unlock(&tx_mutex);
    
    return available_space;
}
