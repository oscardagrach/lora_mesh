/**
 * @file work_queue.h
 * @brief Consolidated work queue for application tasks
 *
 * This module provides a single work queue for all application tasks,
 * reducing memory usage and improving coordination between subsystems.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef WORK_QUEUE_H
#define WORK_QUEUE_H

#include <zephyr/kernel.h>

/**
 * @brief Stack size for the application work queue
 */
#define APP_WORK_STACK_SIZE 4096

/**
 * @brief Priority for the application work queue (higher number = lower priority)
 */
#define APP_WORK_QUEUE_PRIORITY 5

/**
 * @brief Application-wide work queue
 */
extern struct k_work_q app_work_q;

/**
 * @brief Initialize the application work queue
 *
 * Sets up the work queue for all application tasks.
 */
void work_queue_init(void);

/**
 * @brief Error handling helper macro
 *
 * Checks if the expression evaluates to a negative value (error),
 * logs the error with the specified message, and returns the error code.
 */
#define CHECK_ERROR(expr, msg, ...) \
    do { \
        int _err = (expr); \
        if (_err < 0) { \
            LOG_ERR(msg ": %d", ##__VA_ARGS__, _err); \
            return _err; \
        } \
    } while (0)

#endif /* WORK_QUEUE_H */
