/**
 * @file led.c
 * @brief LED control for visual feedback
 *
 * This module provides LED control functions for visual feedback
 * of system events like packet transmission and reception.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led);

#include <led.h>
#include <work_queue.h>

/* Get LED specification from device tree */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Default pulse width in milliseconds */
#define LED_DEFAULT_PULSE_MS 50

/* Forward declaration of the work handler */
static void led_off_work_handler(struct k_work *work);

/* Define work item for delayed LED turn-off */
K_WORK_DELAYABLE_DEFINE(led_off_work, led_off_work_handler);

/**
 * @brief Initialize the LED
 *
 * Configures the LED GPIO pin for output.
 *
 * @return 0 on success, negative errno code on failure
 */
int led_init(void)
{
    /* Check if the LED device is ready */
    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("LED GPIO device is not ready");
        return -ENODEV;
    }

    /* Configure the LED pin as output */
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO pin (err %d)", ret);
        return ret;
    }

    LOG_DBG("LED initialized successfully");
    return 0;
}

/**
 * @brief Toggle the LED state
 *
 * Toggles the LED between on and off states.
 */
void led_toggle(void)
{
    /* Toggle the LED pin */
    gpio_pin_toggle_dt(&led);
}

/**
 * @brief Turn the LED on
 *
 * Turns the LED on and cancels any pending automatic turn-off.
 */
void led_on(void)
{
    /* Cancel any pending automatic off so the LED stays on */
    k_work_cancel_delayable(&led_off_work);
    
    /* Turn the LED on */
    gpio_pin_set_dt(&led, 1);
}

/**
 * @brief Turn the LED off
 *
 * Turns the LED off and cancels any pending automatic turn-off.
 */
void led_off(void)
{
    /* Cancel pending work first so we do not re-enter later */
    k_work_cancel_delayable(&led_off_work);
    
    /* Turn the LED off */
    gpio_pin_set_dt(&led, 0);
}

/**
 * @brief Pulse the LED for a specified duration
 *
 * Turns the LED on and schedules it to turn off after the specified
 * duration. Safe to call from ISR context.
 *
 * @param ms Duration in milliseconds to keep the LED on
 */
void led_pulse_ms(uint32_t ms)
{
    /* Turn the LED on */
    gpio_pin_set_dt(&led, 1);
    
    /* Schedule the LED to turn off after the specified duration */
    k_work_schedule_for_queue(&app_work_q, &led_off_work, K_MSEC(ms));
}

/**
 * @brief Pulse the LED for the default duration
 *
 * Turns the LED on and schedules it to turn off after the default
 * duration (LED_DEFAULT_PULSE_MS).
 */
void led_pulse(void)
{
    led_pulse_ms(LED_DEFAULT_PULSE_MS);
}

/**
 * @brief Work handler to turn the LED off
 *
 * This function is called by the work queue when the LED pulse
 * duration has elapsed.
 *
 * @param work Pointer to the work item
 */
static void led_off_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* Turn the LED off */
    gpio_pin_set_dt(&led, 0);
}
