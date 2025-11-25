/**
 * @file main.c
 * @brief Main application entry point for LoRa mesh network
 *
 * Initializes all subsystems and runs the main application loop
 * for sensor data collection and transmission.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
 
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);
 
#include <led.h>
#include <lora.h>
#include <mesh.h>
#include <packet.h>
#include <sensor.h>
#include <work_queue.h>
#include <zephyr/drivers/hwinfo.h>

/* External declaration for dynamic node address */
extern uint32_t node_address;

/**
 * @brief Application entry point
 *
 * Initializes all subsystems in the correct order.
 * Sensor broadcasting is handled by the work queue if sensors are detected.
 *
 * @return 0 on successful completion (should never return)
 */
int main(void)
{
    uint8_t device_id[8] = {0};

    LOG_INF("Welcome! %s", CONFIG_BOARD_TARGET);

    hwinfo_get_device_id(device_id, 8);

    /* Extract last 4 bytes of device ID for node address */
    node_address = (uint32_t)device_id[4] << 24 |
                   (uint32_t)device_id[5] << 16 |
                   (uint32_t)device_id[6] << 8  |
                   (uint32_t)device_id[7];
    
    /* Ensure node address is not zero (fallback for invalid device ID) */
    if (node_address == 0) {
        node_address = 1;
        LOG_WRN("Device ID produced zero node address, using fallback: 0x%08X", node_address);
    }
    
    LOG_INF("Node address: 0x%08X (from device ID last 4 bytes)", node_address);

    /* Initialize the consolidated work queue first */
    work_queue_init();
    LOG_INF("Work queue initialized");

    /* Initialize hardware and subsystems */
    int ret = led_init();
    if (ret != 0) {
        LOG_ERR("LED initialization failed: %d", ret);
        /* LED failure is not critical, continue */
    }
    
    /* Initialize sensors first to detect available hardware */
    sensor_init();
    
    /* Check if any sensors were detected */
    int sensor_available = sensor_count();
    if (sensor_available == 0) {
        LOG_WRN("No sensors detected. Skipping sensor loop.");
    } else {
        LOG_INF("Detected %d sensors", sensor_available);
    }

    /* Initialize communication subsystems - these are critical */
    ret = packet_init();
    if (ret != 0) {
        LOG_ERR("Packet processing initialization failed: %d", ret);
        return ret;
    }
    
    ret = lora_init();
    if (ret != 0) {
        LOG_ERR("LoRa radio initialization failed: %d", ret);
        return ret;
    }
    
    mesh_init();
    LOG_INF("Mesh network initialized with adaptive reliability-based routing");

    /* All work is now handled by the work queue system.
     * Sensor broadcasting is automatically scheduled if sensors are detected.
     * The main thread can sleep indefinitely. */
    LOG_INF("Initialization complete. All tasks running on work queue.");
    k_sleep(K_FOREVER);

    /* Should never reach here */
    return 0;
}
