/*tsl2561.c
 *
 * Nick Allmeyer
 * ECE 544
 * Winter 2025
 * Project 2
 *
 * Functions in this file:
 *
 * Name: void tsl2561_init(XIic *i2c);
 *
 * Description: Powers on the sensor and reads the ID register to verify sensor presence.
 * 				Uses default timing values (Data from light sensor available after 400ms)
 *
 *
 *
 * Name: uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel);
 *
 * Description: Used to read and store the data from a particular channel of the tsl2561
 *
 *
 *
 * Name: float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1);
 *
 * Description: Calculates the lux value based on data in the datasheet
 * 				Uses T, FN, and CL Package formulas as per TA Daniel's instructions
 *
 * Note: xiic_l.h contains XIic_Send() and XIic_Recv
 *
 * */

//Includes
#include "tsl2561.h"
#include "xiic.h"
#include "xil_printf.h"
#include <math.h>
#include "sleep.h"

//Functions
void tsl2561_init(XIic *i2c)
{
	//Return value for XIic sends and receives.
	int status;

	//POWER ON TSL2561
	//Command to access the control register and value to write to power on tsl2561
	uint8_t control_reg = TSL2561_COMMAND_CNTRL_REG;
	uint8_t power_on_val = TSL2561_POWER_UP_VAL;

	//Access control register
	status = XIic_Send(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &control_reg, sizeof(control_reg), XIIC_STOP);
	if (status != sizeof(control_reg))
		{
			xil_printf("Failed to access TSL2561 Control register\r\n");
			xil_printf("No. bytes returned: %d\r\n", status);
			usleep(ONE_SECOND);
		}

	//Write power on value to control register
	status = XIic_Send(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &power_on_val, sizeof(power_on_val), XIIC_STOP);
	if (status != sizeof(power_on_val))
		{
			xil_printf("Failed to power on TSL2561\r\n");
			xil_printf("No. bytes returned: %d\r\n", status);
			usleep(ONE_SECOND);
		}


	//READ ID REGISTER AND PRINT VALUE
	//Variables to store the sensor ID register and number
	uint8_t sensor_id_reg = TSL2561_COMMAND_ID_REG;
	uint8_t id_number = 0xFF; //Initialized to an incorrect value

	//Access the ID register
	status = XIic_Send(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &sensor_id_reg, sizeof(sensor_id_reg), XIIC_STOP);
	if (status != sizeof(sensor_id_reg))
		{
			xil_printf("Failed to access TSL2561 ID register\r\n");
			xil_printf("No. bytes returned: %d\r\n", status);
			usleep(ONE_SECOND);
		}

	//Get the data from the ID register
	status = XIic_Recv(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &id_number, sizeof(id_number), XIIC_STOP);
	if (status != sizeof(id_number))
			{
				xil_printf("Failed to get ID register data\r\n");
				xil_printf("No. bytes returned: %d\r\n", status);
				usleep(ONE_SECOND);
			}


	//Handle the register ID data and print to the terminal
	xil_printf("TSL2561 Sensor ID Number: %d\r\n", id_number); //should be 0x80
	usleep(ONE_SECOND);
}


uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel)
{
	//result to be returned
	uint16_t channel_data;

	//Variables containing commands to access channel registers
	uint8_t ch_0_data_low = TSL2561_COMMAND_CH0_DATA_LOW_REG;
	uint8_t ch_0_data_high = TSL2561_COMMAND_CH0_DATA_HIGH_REG;
	uint8_t ch_1_data_low = TSL2561_COMMAND_CH1_DATA_LOW_REG;
	uint8_t ch_1_data_high = TSL2561_COMMAND_CH1_DATA_HIGH_REG;

	//Parse for the selected channel and calculate the channel data
	if (channel == TSL2561_CHANNEL_0)
	{
		channel_data = get_channel_data(i2c, ch_0_data_low, ch_0_data_high, channel);
	}

	else if (channel == TSL2561_CHANNEL_1)
	{
		channel_data = get_channel_data(i2c, ch_1_data_low, ch_1_data_high, channel);
	}

	else //This should never happen, channel must be 0 or 1
	{
		xil_printf("ERROR: Invalid channel selection\r\n");
		xil_printf("Returning value of 0\r\n");
		usleep(ONE_SECOND);
		return 0x0000;
	}

	return channel_data;
}


float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1)
{
	//Uses T, FN, and CL Package equations

	//check for division by 0
	if (ch0 == 0)
	{
		xil_printf("ERROR: Channel 0 had a value of 0\r\n");
		xil_printf("Returning a value of 0\r\n");
		usleep(ONE_SECOND);
		return 0.0;
	}

	//Value of divided channels
	float channel_div = (float) ch1/ (float) ch0; //typecast to float

	//Lux return value
	float lux_value;

	if (channel_div > 0 && channel_div <= 0.50)
	{
		lux_value = 0.0304 * ch0 - 0.062 * ch0 * pow(channel_div, 1.4);
	}
	else if (channel_div > 0.50 && channel_div <= 0.61)
	{
		lux_value = 0.0224 * ch0 - 0.031 * ch1;
	}
	else if (channel_div > 0.61 && channel_div <= 0.80)
	{
		lux_value = 0.0128 * ch0 - 0.0153 * ch1;
	}
	else if (channel_div > 0.80 && channel_div <= 1.30)
	{
		lux_value = 0.00146 * ch0 - 0.00112 * ch1;
	}
	else if (channel_div > 1.30)
	{
		lux_value = 0.0;
	}
	else
	{
		lux_value = 0.0; //bad value
	}

	//Print and return lux value
	//Must be cast to int to print
	xil_printf("Lux value (int): %d\r\n", (int) lux_value);
	return lux_value;
}





//Helper function for getting channel data
uint16_t get_channel_data(XIic *i2c, uint8_t channel_data_low_reg, uint8_t channel_data_high_reg, tsl2561_channel_t channel_num)
{
	int status; //store the result of XIic Send and Recv
	uint8_t data_low;
	uint8_t data_high;
	uint16_t channel_data; //result to be returned

	//GET DATA FROM CHANNEL DATA LOW REGISTER
	//Access data low register for given channel
	status = XIic_Send(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &channel_data_low_reg, sizeof(channel_data_low_reg), XIIC_STOP);
	if (status != sizeof(channel_data_low_reg))
		{
			xil_printf("Failed to select DATA LOW Register for channel: %d\r\n", channel_num);
			xil_printf("No. bytes returned: %d\r\n", status);
			xil_printf("Returning value of 0\r\n");
			return 0x0;
		}
	//get data
	status = XIic_Recv(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &data_low, sizeof(data_low), XIIC_STOP);
	if (status != sizeof(data_low))
		{
			xil_printf("Failed to get DATA LOW data for channel: %d\r\n", channel_num);
			xil_printf("No. bytes returned: %d\r\n", status);
			xil_printf("Returning value of 0\r\n");
			return 0x0;
		}

	//GET DATA FROM CHANNEL DATA HIGH REGISTER
	//Access data high register for given channel
	status = XIic_Send(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &channel_data_high_reg, sizeof(channel_data_high_reg), XIIC_STOP);
	if (status != sizeof(channel_data_high_reg))
		{
			xil_printf("Failed to select DATA HIGH Register for channel: %d\r\n", channel_num);
			xil_printf("No. bytes returned: %d\r\n", status);
			xil_printf("Returning value of 0\n");
			return 0x0;
		}
	//get data
	status = XIic_Recv(i2c->BaseAddress, TSL2561_SLAVE_ADDRESS, &data_high, sizeof(data_high), XIIC_STOP);
	if (status != sizeof(data_high))
		{
			xil_printf("Failed to get DATA HIGH data for channel: %d\r\n", channel_num);
			xil_printf("No. bytes returned: %d\r\n", status);
			xil_printf("Returning value of 0\n");
			return 0x0;
		}

	//Calculate and return the channel data as per the datasheet's specifications
	channel_data = 256 * data_low + data_high;
	return channel_data;
}


