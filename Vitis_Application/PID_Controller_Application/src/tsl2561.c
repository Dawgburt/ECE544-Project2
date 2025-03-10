/**
 * @file TSL2561.c
 * @brief Implementation file for TSL2561 luminosity sensor driver.
 *
 * This file contains the function definitions for initializing, reading data,
 * and calculating lux values from the TSL2561 sensor.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author
 * @date 2025-02-20
 *
 * @Notes
 * 20-FEB-2025  PN  Started project
 */

#include "tsl2561.h"
#include "xil_printf.h"
#include "sleep.h"
#include <math.h>

// TSL2561 I2C Address (default 0x39, can also be 0x29 or 0x49)
#define TSL2561_ADDR  0x39

// TSL2561 Register Addresses
#define TSL2561_COMMAND_BIT  0x80
#define TSL2561_CONTROL_REG  0x00
#define TSL2561_TIMING_REG   0x01
#define TSL2561_DATA0_LOW    0x0C
#define TSL2561_DATA0_HIGH   0x0D
#define TSL2561_DATA1_LOW    0x0E
#define TSL2561_DATA1_HIGH   0x0F

// TSL2561 Power On/Off
#define TSL2561_POWER_ON  0x03
#define TSL2561_POWER_OFF 0x00

// Function Prototypes (Local)
static int tsl2561_write_register(XIic *i2c, u8 reg, u8 data);
static int tsl2561_read_register(XIic *i2c, u8 reg, u8 *data);
static int tsl2561_read_channel(XIic *i2c, u8 reg_low, u8 reg_high);

// === Sensor Initialization ===
void tsl2561_init(XIic *i2c) {
    int status;

    // Power on the sensor
    status = tsl2561_write_register(i2c, TSL2561_COMMAND_BIT | TSL2561_CONTROL_REG, TSL2561_POWER_ON);
    if (status != XST_SUCCESS) {
        xil_printf("TSL2561: Failed to power on sensor\n");
    }

    // Configure the timing register (default settings)
    status = tsl2561_write_register(i2c, TSL2561_COMMAND_BIT | TSL2561_TIMING_REG, 0x02); // 402ms integration time
    if (status != XST_SUCCESS) {
        xil_printf("TSL2561: Failed to configure timing register\n");
    }

    xil_printf("TSL2561: Initialization complete\n");
}

// === Read Lux Sensor Data ===
float tsl2561_readLux(XIic *i2c) {
    int ch0 = tsl2561_read_channel(i2c, TSL2561_DATA0_LOW, TSL2561_DATA0_HIGH);
    int ch1 = tsl2561_read_channel(i2c, TSL2561_DATA1_LOW, TSL2561_DATA1_HIGH);

    if (ch0 == -1 || ch1 == -1) {
        xil_printf("TSL2561: Failed to read sensor data\n");
        return -1.0;
    }

    // Convert raw data to Lux (Approximate Calculation)
    float ratio = (float)ch1 / (float)ch0;
    float lux = 0.0;

    if (ratio <= 0.5) {
        lux = (0.0304 * ch0) - (0.062 * ch0 * pow(ratio, 1.4));
    } else if (ratio <= 0.61) {
        lux = (0.0224 * ch0) - (0.031 * ch1);
    } else if (ratio <= 0.80) {
        lux = (0.0128 * ch0) - (0.0153 * ch1);
    } else if (ratio <= 1.30) {
        lux = (0.00146 * ch0) - (0.00112 * ch1);
    } else {
        lux = 0.0;
    }

    return lux;
}

// === Write to a Register ===
static int tsl2561_write_register(XIic *i2c, u8 reg, u8 data) {
    u8 buffer[2] = { reg, data };
    int status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, buffer, 2, XIIC_STOP);
    return (status == 2) ? XST_SUCCESS : XST_FAILURE;
}

// === Read from a Register ===
static int tsl2561_read_register(XIic *i2c, u8 reg, u8 *data) {
    int status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, &reg, 1, XIIC_REPEATED_START);
    if (status != 1) return XST_FAILURE;

    status = XIic_Recv(i2c->BaseAddress, TSL2561_ADDR, data, 1, XIIC_STOP);
    return (status == 1) ? XST_SUCCESS : XST_FAILURE;
}

// === Read 16-bit Channel Data (Low + High) ===
static int tsl2561_read_channel(XIic *i2c, u8 reg_low, u8 reg_high) {
    u8 low, high;
    if (tsl2561_read_register(i2c, TSL2561_COMMAND_BIT | reg_low, &low) != XST_SUCCESS) return -1;
    if (tsl2561_read_register(i2c, TSL2561_COMMAND_BIT | reg_high, &high) != XST_SUCCESS) return -1;
    return ((high << 8) | low);
}
