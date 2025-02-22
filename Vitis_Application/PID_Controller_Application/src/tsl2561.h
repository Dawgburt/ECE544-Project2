/**
 * @file TSL2561.h
 * @brief Header file for TSL2561 luminosity sensor driver.
 *
 * This file contains function prototypes and macros for interfacing with
 * the TSL2561 sensor. It provides initialization, sensor reading, and
 * lux calculation functions.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author
 * @date 2025-02-20
 *
 * @Notes
 * 20-FEB-2025	PN	Started project
 */

#ifndef TSL2561_H
#define TSL2561_H

#include <stdint.h>

/**
 * @brief Initializes the TSL2561 sensor.
 *
 * Powers on the sensor and reads the ID register to verify the sensor's presence.
 * Configures the timing register with either default or user-defined values.
 *
 * @return int Returns 0 on success, or a negative error code on failure.
 */
int tsl2561_init(void);

/**
 * @brief Reads data from a specific channel of the TSL2561 sensor.
 *
 * Issues a command to access the control register and retrieves the light intensity
 * from the specified channel.
 *
 * @param channel The channel number to read (0 for visible+IR, 1 for IR only).
 * @param data Pointer to store the retrieved sensor data.
 * @return int Returns 0 on success, or a negative error code on failure.
 */
int tsl2561_readChannel(int channel, uint16_t *data);

/**
 * @brief Calculates the lux value from the TSL2561 sensor readings.
 *
 * Uses predefined coefficients and equations from the TSL2561 datasheet to
 * convert raw sensor readings into a lux value.
 *
 * @param channel0 Raw data from channel 0 (visible + IR).
 * @param channel1 Raw data from channel 1 (IR only).
 * @return float The calculated lux value.
 */
float tsl2561_calculateLux(uint16_t channel0, uint16_t channel1);

#endif // TSL2561_H
