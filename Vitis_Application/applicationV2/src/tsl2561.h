/**
 * @file tsl2561.h
 * @brief Header file for TSL2561 luminosity sensor driver.
 *
 * This file provides the function prototypes, macros, and data types
 * necessary for interfacing with the TSL2561 luminosity sensor over I2C.
 *
 * @author Phil Nevins
 * @date 2025-02-21
 */

#ifndef TSL2561_H
#define TSL2561_H

#include "xiic.h"
#include <stdint.h>

// Enum for specifying TSL2561 sensor channels
typedef enum {
    TSL2561_CHANNEL_0 = 0,  ///< Full spectrum (visible + IR)
    TSL2561_CHANNEL_1 = 1   ///< Infrared (IR) only
} tsl2561_channel_t;

// Function Prototypes
void tsl2561_init(XIic *i2c);
uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel);
float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1);
float tsl2561_readLux(XIic *i2c);

#endif /* TSL2561_H */
