/**
 * @file led.h
 * @brief LED control interface for activity indication
 *
 * This module provides functions to control an LED for indicating
 * system activity, such as packet transmission/reception.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#ifndef LED_H
#define LED_H
 
#include <zephyr/drivers/gpio.h>

/**
 * @brief Initialize the LED subsystem
 *
 * Configures the GPIO pin for the LED and initializes
 * the work item for delayed LED control.
 *
 * @return 0 on success, negative errno code on failure
 */
int led_init(void);

/**
 * @brief Toggle the LED state
 *
 * Switches the LED from on to off or off to on.
 */
void led_toggle(void);

/**
 * @brief Turn the LED on
 *
 * Turns the LED on and cancels any pending automatic off.
 */
void led_on(void);

/**
 * @brief Turn the LED off
 *
 * Turns the LED off and cancels any pending automatic off.
 */
void led_off(void);

/**
 * @brief Pulse the LED for the default duration
 *
 * Turns the LED on and schedules it to turn off after
 * the default pulse duration (typically 50ms).
 */
void led_pulse(void);

/**
 * @brief Pulse the LED for a specified duration
 *
 * Turns the LED on and schedules it to turn off after
 * the specified duration.
 *
 * @param ms Duration in milliseconds to keep the LED on
 */
void led_pulse_ms(uint32_t ms);
 
#endif /* LED_H */
