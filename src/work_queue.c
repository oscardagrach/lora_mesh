/**
 * @file work_queue.c
 * @brief Consolidated work queue for application tasks
 *
 * This module provides a single work queue for all application tasks,
 * reducing memory usage and improving coordination between subsystems.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <work_queue.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(work_queue);

/* Define a single work queue for the application */
K_THREAD_STACK_DEFINE(app_work_stack, APP_WORK_STACK_SIZE);
struct k_work_q app_work_q;

/**
 * @brief Initialize the application work queue
 *
 * Sets up the work queue for all application tasks.
 */
void work_queue_init(void)
{
    k_work_queue_init(&app_work_q);
    k_work_queue_start(&app_work_q, app_work_stack, APP_WORK_STACK_SIZE, 
                      APP_WORK_QUEUE_PRIORITY, NULL);
    LOG_INF("Application work queue initialized");
}
