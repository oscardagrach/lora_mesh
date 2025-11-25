/**
 * @file sensor.c
 * @brief Sensor management for data collection and transmission
 *
 * This module handles sensor detection, data collection, and transmission
 * of sensor readings over the LoRa mesh network.
 *
 * Copyright (c) 2024 Ryan Grachek
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <string.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor, CONFIG_LOG_DEFAULT_LEVEL);

#include <lora.h>
#include <mesh.h>
#include <packet.h>
#include <sensor.h>
#include <work_queue.h>
#include <config.h>

/**
 * @brief Sensor information structure
 */
typedef struct {
    const struct device *dev;                     /**< Sensor device pointer */
    enum sensor_channel chan[SENSOR_MAX_CHANNELS]; /**< Supported channels */
    uint8_t chan_count;                           /**< Number of supported channels */
} sensor_info_t;

/* Array of registered sensors */
static sensor_info_t sensor_list[SENSOR_MAX_DEVICES];
static int sensor_list_len = 0;

/* Delayable work item for periodic sensor broadcasting */
static struct k_work_delayable sensor_broadcast_work;

#define SLAB_ALIGN 4

/* Memory slabs for efficient allocation/deallocation */
K_MEM_SLAB_DEFINE(sensor_reading_slab, ROUND_UP(sizeof(sensor_t) * SENSOR_MAX_READINGS, SLAB_ALIGN), 2, SLAB_ALIGN);
K_MEM_SLAB_DEFINE(packet_slab, ROUND_UP(MAX_PACKET_LEN, SLAB_ALIGN), 2, SLAB_ALIGN);

/**
 * @brief Get human-readable name for sensor channel
 *
 * @param chan Sensor channel enum value
 * @return Pointer to string with channel name and unit
 */
static const char *chan_name(enum sensor_channel chan)
{
    switch (chan) {
        case SENSOR_CHAN_AMBIENT_TEMP: return "Temperature (°F)";
        case SENSOR_CHAN_PRESS:        return "Pressure (inHg)";
        case SENSOR_CHAN_HUMIDITY:     return "Humidity (%)";
        case SENSOR_CHAN_LIGHT:        return "Light (lux)";
        case SENSOR_CHAN_GAS_RES:      return "VOC (Ohms)";
        case SENSOR_CHAN_ACCEL_X:      return "Accel X (m/s²)";
        case SENSOR_CHAN_ACCEL_Y:      return "Accel Y (m/s²)";
        case SENSOR_CHAN_ACCEL_Z:      return "Accel Z (m/s²)";
        default:                       return "Unknown Channel";
    }
}

/**
 * @brief Register a sensor device
 *
 * Attempts to register a sensor device by checking which
 * channels it supports and adding it to the sensor list.
 *
 * @param dev Device pointer to the sensor
 * @return 0 on success, negative errno code on failure
 */
static int __attribute__((used)) register_sensor(const struct device *dev)
{
    if (!device_is_ready(dev)) {
        LOG_WRN("Device %s not ready", dev->name);
        return -ENODEV;
    }
    
    if (sensor_list_len >= SENSOR_MAX_DEVICES) {
        LOG_WRN("Maximum number of sensors reached, skipping %s",
                dev->name);
        return -ENOMEM;
    }
    
    sensor_info_t *info = &sensor_list[sensor_list_len];
    info->dev = dev;
    
    /* Define common sensor channels to check for support */
    #define COMMON_CHANNELS \
        X(SENSOR_CHAN_AMBIENT_TEMP) \
        X(SENSOR_CHAN_PRESS) \
        X(SENSOR_CHAN_HUMIDITY) \
        X(SENSOR_CHAN_GAS_RES) \
        X(SENSOR_CHAN_LIGHT) \
        X(SENSOR_CHAN_ACCEL_X) \
        X(SENSOR_CHAN_ACCEL_Y) \
        X(SENSOR_CHAN_ACCEL_Z)

    /* Create array of channels to check */
    #define X(chan) chan,
    enum sensor_channel chans[] = {
        COMMON_CHANNELS
    };
    #undef X
    
    int valid_chan = 0;
    struct sensor_value dummy;
    LOG_DBG("Checking channels for %s", dev->name);
    
    /* Try to read each channel to see if it's supported */
    for (int i = 0; i < ARRAY_SIZE(chans) && valid_chan < SENSOR_MAX_CHANNELS; i++) {
        if (sensor_channel_get(dev, chans[i], &dummy) == 0) {
            info->chan[valid_chan] = chans[i];
            LOG_DBG("  Channel %d supported: %s", chans[i],
                    chan_name(chans[i]));
            valid_chan++;
        }
    }
    
    info->chan_count = valid_chan;
    if (valid_chan > 0) {
        sensor_list_len++;
        LOG_INF("Registered %s with %d channels", dev->name, valid_chan);
        return 0;
    } else {
        LOG_WRN("No supported channels found for %s", dev->name);
        return -ENOTSUP;
    }
}

/**
 * @brief Work handler for periodic sensor broadcasting
 *
 * This function is called by the work queue to broadcast sensor data.
 * It calls the sensor_broadcast() function and reschedules itself for
 * the next broadcast interval.
 *
 * @param work Pointer to the work structure
 */
static void sensor_broadcast_work_handler(struct k_work *work)
{
	/* Broadcast sensor data */
	sensor_broadcast();

	/* Reschedule for next broadcast */
	k_work_schedule_for_queue(&app_work_q, &sensor_broadcast_work,
				  K_MSEC(SENSOR_BROADCAST_INTERVAL_MS));
}

/**
 * @brief Initialize the sensor subsystem
 *
 * Detects and registers all compatible sensors connected to the device.
 * If sensors are found, initializes and schedules periodic broadcasting.
 */
void sensor_init(void)
{
    sensor_list_len = 0;

    LOG_INF("Initializing sensors...");

    /* Define a helper macro to register a sensor by compatible string */
    #define REGISTER_BY_COMPAT(compat) \
        do { \
            if (DT_HAS_COMPAT_STATUS_OKAY(compat)) { \
                const struct device *dev = DEVICE_DT_GET_ANY(compat); \
                if (dev != NULL) { \
                    register_sensor(dev); \
                } \
            } \
        } while (0)

    /* Define a list of supported sensor compatibles */
    #define SENSOR_COMPAT_LIST \
        X(bosch_bme680) \
        X(bosch_bme688) \
        X(bosch_bme280) \
        X(adi_adt7420) \
        X(adi_adxl345) \
        X(rohm_bh1750) \
        X(microchip_mcp9808)
        /* Add more sensor compatibles here as needed */

    /* Register all sensors in the compatibility list */
    #define X(compat) REGISTER_BY_COMPAT(compat);
    SENSOR_COMPAT_LIST
    #undef X

    /* Undefine the helper macro to avoid namespace pollution */
    #undef REGISTER_BY_COMPAT

    LOG_INF("Sensor initialization complete: %d sensor devices registered",
            sensor_list_len);

    /* Initialize and schedule periodic broadcasting if sensors are available */
    if (sensor_list_len > 0) {
        k_work_init_delayable(&sensor_broadcast_work, sensor_broadcast_work_handler);
        k_work_schedule_for_queue(&app_work_q, &sensor_broadcast_work,
                                 K_MSEC(SENSOR_BROADCAST_INTERVAL_MS));
        LOG_INF("Sensor broadcasting scheduled every %d ms", SENSOR_BROADCAST_INTERVAL_MS);
    } else {
        LOG_INF("No sensors detected, operating in relay-only mode");
    }
}

/**
 * @brief Collect readings from all registered sensors
 *
 * @param out Array to store sensor readings
 * @param max Maximum number of readings to collect
 * @return Number of readings collected
 */
static int sensor_collect(sensor_t *out, size_t max)
{
    int out_index = 0;

    if (out == NULL) {
        LOG_ERR("NULL output buffer provided to sensor_collect");
        return 0;
    }

    LOG_DBG("Collecting sensor data from %d sensors", sensor_list_len);

    /* Iterate through all registered sensors */
    for (int i = 0; i < sensor_list_len && out_index < max; i++) {
        const struct device *dev = sensor_list[i].dev;

        /* Fetch fresh samples from the sensor */
        int ret = sensor_sample_fetch(dev);
        if (ret != 0) {
            LOG_WRN("Failed to fetch samples from %s: %d", dev->name, ret);
            continue;
        }

        /* Read each supported channel */
        for (int j = 0; j < sensor_list[i].chan_count && out_index < max; j++) {
            struct sensor_value val;
            enum sensor_channel chan = sensor_list[i].chan[j];

            ret = sensor_channel_get(dev, chan, &val);
            if (ret == 0) {
                out[out_index].chan = chan;
                
                /* Convert to float and apply unit conversions in one step */
                float value;
                if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
                    /* °C to °F conversion: (°C * 1.8) + 32 */
                    value = sensor_value_to_float(&val) * 1.8f + 32.0f;
                } else if (chan == SENSOR_CHAN_PRESS) {
                    /* kPa to inHg conversion: kPa * 0.2953 */
                    value = sensor_value_to_float(&val) * 0.2953f;
                } else {
                    /* No conversion needed */
                    value = sensor_value_to_float(&val);
                }

                out[out_index].value = value;
                LOG_DBG("  %s: %.2f", chan_name(chan), (double)value);
                out_index++;
            } else {
                LOG_WRN("Failed to read channel %d from %s: %d", 
                        chan, dev->name, ret);
            }
        }
    }

    LOG_DBG("Collected %d sensor readings", out_index);
    return out_index;
}

/**
 * @brief Print a sensor reading to the log
 *
 * @param sensor Pointer to the sensor reading
 */
static void sensor_print(const sensor_t *sensor)
{
    if (sensor == NULL) {
        LOG_ERR("NULL sensor provided to sensor_print");
        return;
    }

    LOG_INF("%s: %.2f", chan_name(sensor->chan), (double)sensor->value);
}

/**
 * @brief Broadcast sensor readings over the mesh network
 *
 * Collects readings from all registered sensors and broadcasts them
 * to all neighbors using a single mesh broadcast packet. This is more
 * efficient than sending individual packets to each neighbor, as it
 * leverages the broadcast nature of radio communications while still
 * maintaining reliability through ACKs from each neighbor.
 */
void sensor_broadcast(void)
{
    sensor_t *readings;
    int ret;
    
    /* Allocate memory for sensor readings */
    ret = k_mem_slab_alloc(&sensor_reading_slab, (void **)&readings, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Failed to allocate memory for sensor readings: %d", ret);
        return;
    }
    
    /* Clear the sensor readings buffer */
    memset(readings, 0, sizeof(sensor_t) * SENSOR_MAX_READINGS);
    
    /* Collect sensor readings */
    int count = sensor_collect(readings, SENSOR_MAX_READINGS);
    if (count == 0) {
        LOG_WRN("No sensor readings to broadcast");
        k_mem_slab_free(&sensor_reading_slab, readings);
        return;
    }
    
    LOG_INF("Broadcasting %d sensor readings to neighbors...", count);
    
    /* Send using mesh broadcast - even if there are no neighbors, this is the preferred method */
    ret = mesh_send_broadcast((uint8_t *)readings, sizeof(sensor_t) * count);
    
    if (ret != 0) {
        /* Log the error but don't fall back to traditional broadcast */
        LOG_WRN("Mesh broadcast returned: %d (possibly no neighbors available)", ret);
    }
    
    /* Free allocated memory */
    k_mem_slab_free(&sensor_reading_slab, readings);
}

/**
 * @brief Process a received sensor packet
 *
 * Parses and displays sensor readings received from another node.
 * This function handles both direct TYPE_SENSOR packets and sensor data
 * received via TYPE_MESH packets.
 *
 * @param buf Pointer to the packet buffer (including header)
 */
void sensor_process(uint8_t *buf)
{
    if (buf == NULL) {
        LOG_ERR("NULL buffer provided to sensor_process");
        return;
    }

    hdr_t *hdr = (hdr_t *)buf;
    uint8_t count = 0;
    uint8_t *data_ptr = NULL;
    uint32_t src_addr = 0;
    
    /* Determine packet type and extract sensor data accordingly */
    if (hdr->type == TYPE_SENSOR) {
        /* Traditional sensor packet */
        count = (hdr->size - sizeof(hdr_t)) / sizeof(sensor_t);
        data_ptr = buf + sizeof(hdr_t);
        src_addr = hdr->src;
        
        /* Validate packet size */
        if (hdr->size < sizeof(hdr_t) || 
            count > SENSOR_MAX_READINGS || 
            hdr->size != sizeof(hdr_t) + (count * sizeof(sensor_t))) {
            LOG_ERR("Invalid sensor packet size: %u", hdr->size);
            return;
        }
    } else if (hdr->type == TYPE_MESH) {
        /* Sensor data received via mesh network */
        mesh_t *mesh = (mesh_t *)(buf + sizeof(hdr_t));
        
        /* Process both MESH_DATA and MESH_BROADCAST packets */
        if (mesh->type != MESH_DATA && mesh->type != MESH_BROADCAST) {
            LOG_WRN("Received non-data mesh packet in sensor_process (type: %d)", mesh->type);
            return;
        }
        
        /* Calculate sensor reading count from payload size */
        uint16_t payload_size = hdr->size - sizeof(hdr_t) - sizeof(mesh_t);
        count = payload_size / sizeof(sensor_t);
        data_ptr = buf + sizeof(hdr_t) + sizeof(mesh_t);
        src_addr = mesh->orig_src;
        
        /* Validate payload size */
        if (payload_size % sizeof(sensor_t) != 0 || count > SENSOR_MAX_READINGS) {
            LOG_ERR("Invalid sensor data size in mesh packet: %u", payload_size);
            return;
        }
    } else {
        LOG_ERR("Invalid packet type for sensor_process: %u", hdr->type);
        return;
    }
    
    if (count == 0) {
        LOG_WRN("No sensor readings in packet");
        return;
    }
    
    /* Allocate memory for sensor readings */
    sensor_t *readings;
    int ret = k_mem_slab_alloc(&sensor_reading_slab, (void **)&readings, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Failed to allocate memory for sensor readings: %d", ret);
        return;
    }
    
    /* Copy sensor data to our buffer */
    memcpy(readings, data_ptr, count * sizeof(sensor_t));

    LOG_INF("== Sensor readings from node 0x%08X ==", src_addr);
    for (int i = 0; i < count; i++) {
        sensor_print(&readings[i]);
    }
    LOG_INF("==============================");
    
    /* Free allocated memory */
    k_mem_slab_free(&sensor_reading_slab, readings);
}

/**
 * @brief Get the number of registered sensors
 *
 * @return Number of sensors that were successfully registered
 */
int sensor_count(void)
{
    return sensor_list_len;
}
