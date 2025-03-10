/**
 * @file tsl2561.h
 * @brief Header file for TSL2561 luminosity sensor driver.
 *
 * This file provides the function prototypes, macros, and data types
 * necessary for interfacing with the TSL2561 luminosity sensor over I2C.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author Nick A (nick.allmeyer@pdx.edu)
 *
 * @date 2025-02-21
 *
 * @notes
 * Doxygen comments generated via ChatGPT.
 */

#ifndef TSL2561_H
#define TSL2561_H

#include "xiic.h"
#include <stdint.h>

/**
 * @enum tsl2561_channel_t
 * @brief Enum for specifying TSL2561 sensor channels.
 */
typedef enum {
    TSL2561_CHANNEL_0 = 0,  ///< Full spectrum (visible + IR)
    TSL2561_CHANNEL_1 = 1   ///< Infrared (IR) only
} tsl2561_channel_t;

/**
 * @brief Initialize the TSL2561 sensor.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 */
void tsl2561_init(XIic *i2c);

/**
 * @brief Read raw sensor data from a given channel.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 * @param channel Channel to read.
 * @return Raw ADC value from the sensor.
 */
uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel);

/**
 * @brief Compute the lux value from raw ADC data.
 *
 * @param ch0 Raw ADC value from channel 0.
 * @param ch1 Raw ADC value from channel 1.
 * @return Computed lux value.
 */
float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1);

/**
 * @brief Read the current lux value from the sensor.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 * @return Computed lux value.
 */
float tsl2561_readLux(XIic *i2c);

#endif /* TSL2561_H */
