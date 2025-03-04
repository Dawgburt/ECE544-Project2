/*tsl2561.h
 *
 * Nick Allmeyer
 * ECE 544
 * Winter 2025
 * Project 2
 *
 * Contains structures, enums, constants, and function prototypes for tsl2561.c
 * */

//Includes
#include <stdint.h>
#include "xiic.h"

//Constants
#define TSL2561_SLAVE_ADDRESS 0x39
#define TSL2561_COMMAND_CNTRL_REG 0x80
#define TSL2561_POWER_UP_VAL 0x03 //Write to Control Reg (address 0x00)
#define TSL2561_COMMAND_CH0_DATA_LOW_REG 0x8C
#define TSL2561_COMMAND_CH0_DATA_HIGH_REG 0x8D
#define TSL2561_COMMAND_CH1_DATA_LOW_REG 0x8E
#define TSL2561_COMMAND_CH1_DATA_HIGH_REG 0x8F
#define TSL2561_COMMAND_ID_REG 0x8A
#define SHIFT_ID_BITS 4

#define ONE_SECOND 1000000


//Enums
typedef enum{
	TSL2561_CHANNEL_0 = 0,
	TSL2561_CHANNEL_1 = 1
} tsl2561_channel_t;


//Function Prototypes
void tsl2561_init(XIic *i2c);

uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel);

float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1);

//Helper function for getting channel data
uint16_t get_channel_data(XIic *i2c, uint8_t channel_data_low_reg, uint8_t channel_data_high_reg, tsl2561_channel_t channel_num);
