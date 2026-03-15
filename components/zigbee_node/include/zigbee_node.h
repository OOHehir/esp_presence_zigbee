#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ld2410c.h"
#include "vl53l0x.h"

/**
 * Initialise the Zigbee stack and register the device endpoint.
 * Must be called before zigbee_node_start().
 */
esp_err_t zigbee_node_init(void);

/**
 * Start the Zigbee main loop. This function does not return.
 * Handles network joining, commissioning, and stack events.
 * @param sensor_task_fn  Task function to launch once Zigbee stack is ready.
 */
void zigbee_node_start(TaskFunction_t sensor_task_fn);

/**
 * Update LD2410C sensor data on the Zigbee endpoint.
 * Thread-safe — acquires the Zigbee lock internally.
 * Updates Analog Input on endpoint 1 (presence as 0.0/1.0).
 */
esp_err_t zigbee_node_update_ld2410c(const ld2410c_data_t *data);

/**
 * Update VL53L0X sensor data on the Zigbee endpoint.
 * Thread-safe — acquires the Zigbee lock internally.
 * Updates Analog Input on endpoint 2 (range in cm as float).
 */
esp_err_t zigbee_node_update_vl53l0x(const vl53l0x_data_t *data);

/**
 * Update LD2410C static energy on the Zigbee endpoint.
 * Thread-safe — acquires the Zigbee lock internally.
 * Updates Analog Input on endpoint 4 (static_energy as 0–100 float).
 */
esp_err_t zigbee_node_update_static_energy(uint8_t static_energy);
