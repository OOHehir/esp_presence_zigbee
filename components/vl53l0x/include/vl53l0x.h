#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

typedef struct {
    uint16_t range_cm;
    uint8_t  status;   /* 0=valid, 1=sigma fail, 2=signal fail, 255=no target */
} vl53l0x_data_t;

/**
 * Initialise the VL53L0X sensor on an I2C bus.
 * Creates the I2C master bus and device handle internally.
 * @param port      I2C port number
 * @param sda_pin   GPIO for SDA
 * @param scl_pin   GPIO for SCL
 * @param xshut_pin GPIO for XSHUT (hardware reset), or GPIO_NUM_NC to skip
 */
esp_err_t vl53l0x_init(i2c_port_t port, int sda_pin, int scl_pin, gpio_num_t xshut_pin);

/**
 * Read the latest range measurement.
 * @param out  Filled with range_mm and status on success.
 *             range_cm = VL53L0X_OUT_OF_RANGE_CM for out-of-range/error conditions.
 */
esp_err_t vl53l0x_read(vl53l0x_data_t *out);

/**
 * Shut down the VL53L0X and release I2C resources.
 */
void vl53l0x_deinit(void);

/* --- Status mapping (exposed for unit testing) --- */

/**
 * Map VL53L0X range status register value to simplified status code.
 * @param device_status  Raw status from register 0x14 (upper nibble >> 3)
 * @return 0=valid, 1=sigma fail, 2=signal fail, 255=no target
 */
uint8_t vl53l0x_map_status(uint8_t device_status);

/** Sentinel range value for invalid/out-of-range readings (cm) */
#define VL53L0X_OUT_OF_RANGE_CM 200

/** Maximum reliable range in mm — readings above this are treated as no-target */
#define VL53L0X_MAX_RANGE_MM    1200
