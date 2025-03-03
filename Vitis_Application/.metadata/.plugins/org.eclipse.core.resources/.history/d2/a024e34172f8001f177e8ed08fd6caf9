/**
 * @file tsl2561.c
 * @brief Implementation file for TSL2561 luminosity sensor driver.
 *
 * This file contains the function definitions for initializing, reading data,
 * and calculating lux values from the TSL2561 sensor.
 *
 * @author Phil Nevins
 * @date 2025-02-21
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
#define TSL2561_ID_REG       0x0A
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
static int tsl2561_read_word(XIic *i2c, u8 reg, uint16_t *value);

/**
 * @brief Initialize the TSL2561 sensor.
 *
 * This function powers on the sensor and verifies its presence by reading the ID register.
 * It also configures the timing register for an optimal integration time.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 */
void tsl2561_init(XIic *i2c) {
    xil_printf("Initializing TSL2561 Sensor...\r\n");

    // Reset the I2C controller before starting
    XIic_Reset(i2c);

    // Initialize the sensor (Power ON and configure)
    int status = tsl2561_write_register(i2c, TSL2561_COMMAND_BIT | TSL2561_CONTROL_REG, TSL2561_POWER_ON);
    if (status != XST_SUCCESS) {
        xil_printf("TSL2561: Failed to power on sensor\r\n");
    }

    // Configure the timing register
    status = tsl2561_write_register(i2c, TSL2561_COMMAND_BIT | TSL2561_TIMING_REG, 0x02); // 402ms integration time
    if (status != XST_SUCCESS) {
        xil_printf("TSL2561: Failed to configure timing register\r\n");
    }

    xil_printf("TSL2561: Initialization complete\r\n");
}


/**
 * @brief Read raw sensor data from a given channel (CH0 or CH1).
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 * @param channel Channel to read (TSL2561_CHANNEL_0 or TSL2561_CHANNEL_1).
 * @return 16-bit raw ADC value from the sensor, or 0xFFFF on failure.
 */
uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel) {
    uint16_t value = 0xFFFF;  // Default error value
    u8 reg_low, reg_high;

    // Select registers based on the channel
    if (channel == TSL2561_CHANNEL_0) {
        reg_low = TSL2561_DATA0_LOW;
        reg_high = TSL2561_DATA0_HIGH;
    } else {
        reg_low = TSL2561_DATA1_LOW;
        reg_high = TSL2561_DATA1_HIGH;
    }

    // Read 16-bit data from the selected channel
    if (tsl2561_read_word(i2c, TSL2561_COMMAND_BIT | reg_low, &value) != XST_SUCCESS) {
        xil_printf("TSL2561: Failed to read channel %d\r\n", channel);
    }

    return value;
}

/**
 * @brief Convert raw sensor data to lux using the appropriate formula.
 *
 * @param ch0 Raw ADC value from channel 0.
 * @param ch1 Raw ADC value from channel 1.
 * @return Computed lux value.
 */
float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1) {
    if (ch0 == 0) return 0.0;  // Avoid division by zero

    float ratio = (float)ch1 / (float)ch0;
    float lux = 0.0;

    // Apply lux calculation based on TSL2561 datasheet formulas
    if (ratio <= 0.5) {
        lux = (0.0304 * ch0) - (0.062 * ch0 * pow(ratio, 1.4));
    } else if (ratio <= 0.61) {
        lux = (0.0224 * ch0) - (0.031 * ch1);
    } else if (ratio <= 0.80) {
        lux = (0.0128 * ch0) - (0.0153 * ch1);
    } else if (ratio <= 1.30) {
        lux = (0.00146 * ch0) - (0.00112 * ch1);
    }

    // Ensure lux calculation does not overestimate brightness
    if (lux > 1000) lux = 999;  // Clamp max lux to prevent extreme values

    return lux;
}

/**
 * @brief Read the current lux value from the sensor.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 * @return Computed lux value.
 */
float tsl2561_readLux(XIic *i2c) {
    uint16_t ch0 = tsl2561_readChannel(i2c, TSL2561_CHANNEL_0);
    uint16_t ch1 = tsl2561_readChannel(i2c, TSL2561_CHANNEL_1);
    return tsl2561_calculateLux(ch0, ch1);
}

/**
 * @brief Write a value to a TSL2561 register.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 * @param reg Register address.
 * @param data Value to write.
 * @return XST_SUCCESS or XST_FAILURE.
 */
static int tsl2561_write_register(XIic *i2c, u8 reg, u8 data) {
    u8 buffer[2] = { reg, data };
    int status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, buffer, 2, XIIC_STOP);
    return (status == 2) ? XST_SUCCESS : XST_FAILURE;
}

/**
 * @brief Read a single byte from a TSL2561 register.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 * @param reg Register address.
 * @param data Pointer to store the read value.
 * @return XST_SUCCESS or XST_FAILURE.
 */
static int tsl2561_read_register(XIic *i2c, u8 reg, u8 *data) {
    int status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, &reg, 1, XIIC_REPEATED_START);
    if (status != 1) return XST_FAILURE;

    status = XIic_Recv(i2c->BaseAddress, TSL2561_ADDR, data, 1, XIIC_STOP);
    return (status == 1) ? XST_SUCCESS : XST_FAILURE;
}

/**
 * @brief Read a 16-bit word from a TSL2561 register.
 *
 * @param i2c Pointer to the XIic instance for I2C communication.
 * @param reg Register address.
 * @param value Pointer to store the read value.
 * @return XST_SUCCESS or XST_FAILURE.
 */
static int tsl2561_read_word(XIic *i2c, u8 reg, uint16_t *value) {
    u8 low, high;
    if (tsl2561_read_register(i2c, reg, &low) != XST_SUCCESS) return XST_FAILURE;
    if (tsl2561_read_register(i2c, reg + 1, &high) != XST_SUCCESS) return XST_FAILURE;
    *value = ((uint16_t)high << 8) | low;
    return XST_SUCCESS;
}
