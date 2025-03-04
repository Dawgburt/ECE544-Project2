/*
 * Nick Allmeyer
 * ECE 544
 * Winter 2025
 *
 * Project 2
 *
 *The following project utilizes FreeRTOS and a custom written TSL2561 driver to light an LED to a pre-defined
 *brightness and implements a PID algorithm to adhere as close to that set brightness as possible in the face of
 *brightness environmental disturbances such as a vast increase or decrease of light.
 *
 *This version of the application was primarily developed by Nick Allmeyer with some additional help from those
 *credited below, including my project partner Phil Nevins. We decided to work in parallel for this project because we
 *felt we both got more learning by not strictly splitting up the tasks.
 *
 *This application builds upon the given starter code FreeRtosDemo.c and implements the given possible tasking model
 *included in the project assignment. The parse input task simply takes a semaphore given by the button interrupt handler (and it only handles the buttons) and
 *identifies which of the valid (U, D, and R) buttons were pushed. It then reads the switch values, lights the corresponding LEDs (for user feedback)
 *and sends the pressed button value and switch values through a message queue. Parsing input is the second highest priority task as the interrupt
 *and semaphore handling will properly record button values into the queue without interfering with the hefty PID task.
 *
 *The PID task is the highest priority task and also the largest. It gets the pressed button and switch values from the message queue and then reads all the
 *switch values to determine the current control parameters, setting them appropriately. Boolean variables were used here when possible to reduce program size.
 *The task then sets these control values and using the custom driver to read the tsl2561 and get the lux value. The task then executes a simple PID algorithm
 *and converts that to a duty cycle to be written to the RGB1 LED blue channel, white powers on the white led. Finally the task sends an integer conversion of
 *the current setpoint and lux to the Dsiplay task.
 *
 *The display task takes setpoint and lux values and writes them to the seven segment display. It is the lowest priority task.
 *
 *The main loop initializes the platform, peripherals, semaphore, message queue, and interrupt handlers. Most of this was in the provided starter code, with
 *some additions made such as initializing the i2c functionality. It then creates the three tasks and starts the scheduler.
 *
 *Credits:
 *The creation of the project and starter code was provided by professor Victoria Van Gaasbeck, professor Roy Kravitx, and TA Daniel Jacobsen.
 *My project partner Phil Nevins was instrumental in helping me develop my version of this application.
 *Chat GPT was instrumental in helping debug as well as in creating the PID algorithm. It was consulted a lot for general help.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "nexys4IO.h"
#include <stdlib.h>
#include "platform.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"

/* Application includes*/
#include <math.h>
#include "xiic.h"
#include "tsl2561.h"
#include <stdbool.h>

/*Definitions for NEXYS4IO Peripheral*/
#define N4IO_DEVICE_ID		    XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR		    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR		    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

#define BTN_CHANNEL		1
#define SW_CHANNEL		2

#define mainQUEUE_LENGTH					( 4 ) //increased to 4. Ideally never sending more than 2 so this should give some breathing room

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

/* Button MASKS for GPIO input*/
#define GPIO_BTN_U_MASK 0x0008
#define GPIO_BTN_D_MASK 0x0004
#define GPIO_BTN_R_MASK 0x0001
#define GPIO_BTN_L_MASK 0x0002
#define GPIO_BTN_C_MASK 0x0010

/* 20ms debounce delay*/
#define DEBOUNCE_DELAY 20000

/* Individual Switch MASKS*/
#define SW_7_MASK 0x80
#define SW_6_MASK 0x40
#define SW_5_MASK 0x20
#define SW_4_MASK 0x10
#define SW_3_MASK 0x08
#define SW_2_MASK 0x04
#define SW_1_MASK 0x02
#define SW_0_MASK 0x01

#define SW_76_MASK 0xC0 //mask for both sw 7 and sw 6


/* Default values for Setpoint, Kp, Ki, Kd*/
#define DEFAULT_SETPOINT 75.0
#define DEFAULT_Kp 3.14
#define DEFAULT_Ki 0.234
#define DEFAULT_Kd 5.87

#define KP_SIZE 1
#define KI_SIZE 0.1
#define KD_SIZE 0.5

#define PWM_MIN 0.0
#define PWM_MAX 255.0
#define MAX_INTEGRAL 100.0
#define MIN_INTEGRAL 0.0

//Create Instances
static XGpio xInputGPIOInstance;
XIic i2c;

//In event of no queue from pid to display, use these
//global variables to communitcate between PID and DISP
uint16_t global_setpoint;
uint16_t global_lux;

//setpoint
float setpoint = DEFAULT_SETPOINT;
bool first_time_running = true; //flag to initialize Kp, Ki, Kd


//Function Declarations
static void prvSetupHardware( void );


//Declare a Semaphore
xSemaphoreHandle binary_sem;

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;
static xQueueHandle xQueue_Parse_to_PID = NULL;
static xQueueHandle xQueue_PID_to_Disp = NULL;

//ISR, to handle interrupt of GPIO btns
//Give a Semaphore
static void gpio_intr (void *pvUnused)
{
	xSemaphoreGiveFromISR(binary_sem,NULL);

	XGpio_InterruptClear( &xInputGPIOInstance, XGPIO_IR_MASK );

}

//GLOBAL DEBUG
uint32_t rgb_debug = 0x0000FF00; //full green

//Tasks
void Parse_Input_Task(void *p)
{
	uint16_t btn_pressed; //stores the button that has been pressed
	uint16_t button_state; //"vector" containing the pushed buttons
	uint16_t sw_state; //"vector" containing the switches' values
	uint16_t led_value; //"vector" containing the led values

	while(1)
	{
		//initialize button pressed to 0. No button pressed
		btn_pressed = 0x0;
		button_state = 0x0;

		//make sure all leds are low
		if (first_time_running)
		{
			NX4IO_setLEDs((uint32_t) 0x0);
		}


		if(xSemaphoreTake(binary_sem, 100)) //1s
		{
			//Read the button that caused the interrupt
			button_state = XGpio_DiscreteRead(&xInputGPIOInstance, BTN_CHANNEL);
			if ((button_state & GPIO_BTN_U_MASK) == GPIO_BTN_U_MASK) //Check U
			{
				btn_pressed = GPIO_BTN_U_MASK;
				xil_printf("Up Button Pressed\r\n");
				usleep(DEBOUNCE_DELAY);

			}
			else if ((button_state & GPIO_BTN_D_MASK) == GPIO_BTN_D_MASK) //Check D
			{
				btn_pressed = GPIO_BTN_D_MASK;
				xil_printf("Down Button Pressed\r\n");
				usleep(DEBOUNCE_DELAY);

			}
			else if ((button_state & GPIO_BTN_R_MASK) == GPIO_BTN_R_MASK) //Check R
			{
				btn_pressed = GPIO_BTN_R_MASK;
				xil_printf("Right Button Pressed\r\n");
				usleep(DEBOUNCE_DELAY);

			}
			else if ((button_state & GPIO_BTN_C_MASK) == GPIO_BTN_C_MASK) //light up white LED DEBUG
			{
				btn_pressed = GPIO_BTN_C_MASK; //does nothing
				xil_printf("Center Button Pressed\r\n");
				usleep(DEBOUNCE_DELAY);
			}
			else if ((button_state & GPIO_BTN_L_MASK) == GPIO_BTN_L_MASK) //light up white LED DEBUG
			{
				btn_pressed = GPIO_BTN_L_MASK; //does nothing
				xil_printf("Left Button Pressed\r\n");
				usleep(DEBOUNCE_DELAY);
			}
			else
			{
				btn_pressed = 0x0; //default, doesn't result in system behavior change
			}

			//Send buttons and switches
			sw_state = XGpio_DiscreteRead(&xInputGPIOInstance, SW_CHANNEL);
			//update the green leds to reflect our changes (not necessary, but nice for visual feedback)
			led_value = sw_state;
			NX4IO_setLEDs((uint32_t) led_value);

			//Send values to the message queue
			xQueueSend(xQueue_Parse_to_PID, &btn_pressed, portMAX_DELAY);
			xQueueSend(xQueue_Parse_to_PID, &sw_state, portMAX_DELAY);
		}
		else //button not pressed, so 0x0 default val
		{
			//Send buttons and switches
			sw_state = XGpio_DiscreteRead(&xInputGPIOInstance, SW_CHANNEL);
			//update the green leds to reflect our changes (not necessary, but nice for visual feedback)
			led_value = sw_state;
			NX4IO_setLEDs((uint32_t) led_value);

			//Send values to the message queue
			xQueueSend(xQueue_Parse_to_PID, &btn_pressed, portMAX_DELAY);
			xQueueSend(xQueue_Parse_to_PID, &sw_state, portMAX_DELAY);
		}
	}
	//For safety always be able to kill the task
	xil_printf("ERROR: Parse Input Task exited infinite loop. Killing task\r\n");
	vTaskDelete(NULL);
}

void PID_Task(void *p)
{
	//Variables to store messages sent from the queue
	uint16_t button_value;
	uint16_t sw_value;

	//Variables
	//Parameter set varaibles
	bool change_setpoint;
	bool change_Kp;
	bool change_Ki;
	bool change_Kd;
	bool enable_proportional;
	bool enable_integral;
	bool enable_derivative;
	float inc_dec_value;

	//PID control variables
	float Kp, Ki, Kd;

	//TSL2561 Variables
	uint16_t channel_0_data;
	uint16_t channel_1_data;
	float lux_value;

	//PID variables
	float P;
	float I;
	float D;
	float integral;
	float PID_output;
	float error;
	float prev_error;
	//measured value is the lux value returned from the sensor

	//Used to write the duty cycle into the rgb led blue channel
	uint8_t pwm_duty_cycle;


	while(1)
	{
		//xil_printf("PID task start\r\n");
		//Extract variables from the message queue. button first, switches seconds
		xQueueReceive(xQueue_Parse_to_PID, &button_value, portMAX_DELAY);
		xQueueReceive(xQueue_Parse_to_PID, &sw_value, portMAX_DELAY);

		//Kx initialization
		//Kx initialization flag is global, but only ever affected by this if branch in the PID task.
		if (first_time_running)
		{
			Kp = DEFAULT_Kp;
			Ki = DEFAULT_Ki;
			Kd = DEFAULT_Kd;

			//Initialize the PID controls upon first run.
			P = 0.0;
			I = 0.0;
			D = 0.0;
			integral = 0.0;
			PID_output = 0.0;
			error = 0.0;
			prev_error = 0.0;
			//After first run, flag is always set to false
			first_time_running = false;
		}

		//Update control parameters
		//Switches:
		//sw [7:6]: Are we changing the Kx values?
		if ((sw_value & (SW_7_MASK | SW_6_MASK)) == SW_76_MASK) //sw 7 and 6 both high
		{
			change_Kd = true;
		}
		else if ((sw_value & SW_7_MASK) == SW_7_MASK) //7 and 6 are not both high. Just check 7
		{
			change_Ki = true;
		}
		else if ((sw_value & SW_6_MASK) == SW_6_MASK) //Just check 6
		{
			change_Kp = true;
		}
		else //neither are high. Change nothing.
		{
			change_Kp = false;
			change_Ki = false;
			change_Kd = false;
		}

		//sw [5:4]: By how much should we increment/decrement?
		if ((sw_value & SW_5_MASK) == SW_5_MASK) //Is sw 5 high?
		{
			inc_dec_value = 10.0;
		}
		else if ((sw_value & SW_4_MASK) == SW_4_MASK) //sw 5 is false. Check sw 4
		{
			inc_dec_value = 5.0;
		}
		else //sw 5 and 4 both false
		{
			inc_dec_value = 1.0;
		}

		//sw 3: Are we changing the setopint?
		if ((sw_value & SW_3_MASK) == SW_3_MASK) //Is sw 3 high?
		{
			change_setpoint = true;
		}
		else //sw 3 is low
		{
			change_setpoint  = false;
		}

		//sw 2: Enable derivative control in PID?
		if ((sw_value & SW_2_MASK) == SW_2_MASK) //Is sw 2 high?
		{
			enable_derivative = true;
		}
		else //sw 2 is low
		{
			enable_derivative  = false;
		}

		//sw 1: Enable integral control in PID?
		if ((sw_value & SW_1_MASK) == SW_1_MASK) //Is sw 1 high?
		{
			enable_integral = true;
		}
		else //sw 1 is low
		{
			enable_integral  = false;
		}

		//sw 0: Enable proportional control in PID?
		if ((sw_value & SW_0_MASK) == SW_0_MASK) //Is sw 0 high?
		{
			enable_proportional = true;
		}
		else //sw 0 is low
		{
			enable_proportional  = false;
		}


		//Update setpoint, Kp, Ki, Kd as appropriate
		//Setpoint
		if (change_setpoint)
		{
			//Which button was pressed?
			if (button_value == GPIO_BTN_U_MASK)
			{
				setpoint += inc_dec_value;
			}
			else if (button_value == GPIO_BTN_D_MASK)
			{
				setpoint -= inc_dec_value;
			}
		}
		//Kp, Ki, Kd
		//also reset the PID control values
		if (change_Kp) //Kp
		{
			//Which button was pressed?
			if (button_value == GPIO_BTN_U_MASK)
			{
				Kp += (inc_dec_value * KP_SIZE);
			}
			else if (button_value == GPIO_BTN_D_MASK)
			{
				Kp -= (inc_dec_value * KP_SIZE);
			}
		}

		if (change_Ki) //Ki
		{
			//Which button was pressed?
			if (button_value == GPIO_BTN_U_MASK)
			{
				Ki += (inc_dec_value * KI_SIZE);
			}
			else if (button_value == GPIO_BTN_D_MASK)
			{
				Ki -= (inc_dec_value * KI_SIZE);
			}
		}

		if (change_Kd) //Kd
		{
			//Which button was pressed?
			if (button_value == GPIO_BTN_U_MASK)
			{
				Kd += (inc_dec_value * KD_SIZE);
			}
			else if (button_value == GPIO_BTN_D_MASK)
			{
				Kd -= (inc_dec_value * KD_SIZE);
			}
		}

		//RESET HANDLING
		//not changing Kx values but still want to reset
		if (button_value == GPIO_BTN_R_MASK)
		{
			setpoint = DEFAULT_SETPOINT;
			Kp = DEFAULT_Kp;
			Ki = DEFAULT_Ki;
			Kd = DEFAULT_Kd;
		}

		//get lux value using TSL2561 Driver
		channel_0_data = tsl2561_readChannel(&i2c, TSL2561_CHANNEL_0);
		channel_1_data = tsl2561_readChannel(&i2c, TSL2561_CHANNEL_1);
		lux_value  = tsl2561_calculateLux(channel_0_data, channel_1_data);

		//PID////////////////////////////////////////////////////////////////
		//Execute the PID algorithm
		//measured value is the lux value returned from the sensor

		error = setpoint - lux_value;

		//Check if given control methods are enabled
		//Proportional
		if (enable_proportional)
		{
			P = Kp * error;
		}
		else
		{
			P = 0.0;
		}

		//Integral
		if (enable_integral)
		{
			//Handle WIndup prevention
			if (integral > MAX_INTEGRAL)
			{
				integral = MAX_INTEGRAL;
			}
			else if (integral < MIN_INTEGRAL)
			{
				integral  = MIN_INTEGRAL;
			}
			else
			{
				integral += error;
			}

			I = Ki * integral;
		}
		else
		{
			I = 0.0;
		}

		//Derivative
		if (enable_derivative)
		{
			D = Kd * (error - prev_error);
		}
		else
		{
			D = 0.0;
		}

		prev_error = error;
		PID_output = P + I + D;

		//xil_printf("~PID_output: %d\r\n", (int) PID_output); //can't print floats

		//Drive PWM signal for LED
		//Get PWM from PID
		if (PID_output > 255.0)
		{
			pwm_duty_cycle = 255;
		}
		else if (PID_output < 0)
		{
			pwm_duty_cycle = 0;
		}
		else
		{
			pwm_duty_cycle = (uint8_t) ((PID_output / 255.0) * 100); //Always between 0 and 100, so fine for an 8 bit value
		}
		xil_printf("Duty Cycle: %d\r\n", pwm_duty_cycle);

		//RGB1 BLue LED duty cycle is bottom 8 bits of a 32 bit register
		//only writing to the LED blue channel
		NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwm_duty_cycle);

		//Send Message to display thread to update setpoint and current lux
		//send setpoint, then lux. Cast to unit16_t to save space. The seven segment display only displays 3 decimals
		uint16_t send_lux = (uint16_t) lux_value;
		uint16_t send_setpoint = (uint16_t) setpoint;

		xQueueSend(xQueue_PID_to_Disp, &send_setpoint, portMAX_DELAY);
		xQueueSend(xQueue_PID_to_Disp, &send_lux, portMAX_DELAY);
	}
	//For safety always be able to kill the task
	xil_printf("ERROR: PID Task exited infinite loop. Killing task\r\n");
	vTaskDelete(NULL);
}

void Display_Task(void *p)
{

	//Variables to store the messages from the msgQ
	uint16_t setpoint_display_value;
	uint16_t lux_display_value;

	//Stores the hundreds, tens, and ones elements of a  3 digit number
	//used for displaying to SSEG
	uint16_t hundreds_val;
	uint16_t tens_val;
	uint16_t ones_val;


	while(1)
	{
		//Extract variables from the message queue. lux first, setpoint second
		xQueueReceive(xQueue_PID_to_Disp, &setpoint_display_value, portMAX_DELAY);
		xQueueReceive(xQueue_PID_to_Disp, &lux_display_value, portMAX_DELAY);

		//Update the seven segment display
		//Setpoint
		hundreds_val = (setpoint_display_value / 100);
		tens_val = ((setpoint_display_value % 100) / 10);
		ones_val = (setpoint_display_value % 10);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, (enum _NX4IO_charcodes) hundreds_val);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, (enum _NX4IO_charcodes) tens_val);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT5, (enum _NX4IO_charcodes) ones_val);

		//Lux
		hundreds_val = (uint8_t) (lux_display_value / 100);
		tens_val = (uint8_t) ((lux_display_value % 100) / 10);
		ones_val = (uint8_t) (lux_display_value % 10);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, (enum _NX4IO_charcodes) hundreds_val);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT2, (enum _NX4IO_charcodes) tens_val);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, (enum _NX4IO_charcodes) ones_val);

	}
	//For safety always be able to kill the task
	xil_printf("ERROR: Display Task exited infinite loop. Killing task\r\n");
	vTaskDelete(NULL);

}


int main(void)
{
	// Announcement
	xil_printf("ECE 544 Project 2\r\n");

	//Initialize the HW
	prvSetupHardware();

	//Create Semaphore
	vSemaphoreCreateBinary(binary_sem);

	/* Create the queue */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint16_t ) );
	xQueue_Parse_to_PID = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint16_t ) );
	xQueue_PID_to_Disp = xQueueCreate( mainQUEUE_LENGTH, sizeof( float ) );

	/* Sanity check that the queue was created. */
	configASSERT( xQueue );

	//Create Tasks

	//Parse Input Task
	xTaskCreate(Parse_Input_Task,
				( const char * ) "Input_Task",
				512,
				NULL,
				2, //recording inputs is the second highest priority. Semaphore will capture inputs
				NULL);

	//PID Task
	xTaskCreate(PID_Task,
				( const char * ) "PID_Task",
				2048, //Biggest task I have, current number is an educated guess. Fine tune later after finishing task writing.
				NULL,
				3, //Most important task. It is the longest and necessary for the PID functionality
				NULL);

	//Display Task
	xTaskCreate(Display_Task,
				( const char * ) "Disp_Task",
				512,
				NULL,
				1, //updating the display is least important
				NULL);

	//Start the Scheduler
	xil_printf("Starting the scheduler\r\n");
	vTaskStartScheduler();

	return -1;
}


static void prvSetupHardware( void )
{
	uint32_t xStatus;

	const unsigned char ucSetToInput = 0xFFU;

	xil_printf("Initializing GPIO's\r\n");


	/* Initialize the GPIO for the button inputs. */

		xStatus = XGpio_Initialize( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );


		if( xStatus == XST_SUCCESS )
		{
		/* Install the handler defined in this task for the button input.
		*NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
		must be used for this purpose. */
		xStatus = xPortInstallInterruptHandler( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR, gpio_intr, NULL );

		if( xStatus == pdPASS )
		{
			xil_printf("Buttons interrupt handler installed\r\n");

			/* Set switches and buttons to input. */
			XGpio_SetDataDirection( &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
			XGpio_SetDataDirection( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );

			/* Enable the button input interrupts in the interrupt controller.
			*NOTE* The vPortEnableInterrupt() API function must be used for this
			purpose. */

			vPortEnableInterrupt( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

			/* Enable GPIO channel interrupts on button channel. Can moodify to include switches */
			XGpio_InterruptEnable( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
			XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
		}

		//initialize I2C
		//XIic i2c is the instance and exists globally right below the gpio instance
		int xiic_status;
		XIic_Config *xiic_Cfg_Ptr;

		//Lok up config
		xiic_Cfg_Ptr  = XIic_LookupConfig(XPAR_AXI_IIC_0_DEVICE_ID);
		if (xiic_Cfg_Ptr == NULL)
		{
			xil_printf("I2C Config Lookup Failed\r\n");
		}


		//cfg intitialize
		xiic_status = XIic_CfgInitialize(&i2c, xiic_Cfg_Ptr, XPAR_AXI_IIC_0_BASEADDR);
		if (xiic_status != XST_SUCCESS)
		{
			xil_printf("ERROR: I2C initialization failed\r\n");
		}

		//Start I2C
		xiic_status = XIic_Start(&i2c);
		//always returns XST_SUCCESS, no need to check

		//xiic self test
		xiic_status = XIic_SelfTest(&i2c);
		if (xiic_status != XST_SUCCESS)
		{
			xil_printf("ERROR: I2C initialization failed\r\n");
		}
		else //is XST_SUCCESS
		{
			xil_printf("I2C initialization complete\r\n");
		}

		//Initialize I2C
		/*xiic_status = XIic_Initialize(&i2c, XPAR_AXI_IIC_0_DEVICE_ID); //thank you xparameters.h
		if (xiic_status != XST_SUCCESS)
		{
			xil_printf("ERROR: I2C initialization failed\r\n");
			return XST_FAILURE; //results in a warning.
		}*/

		//Initialize the TSL2561
		tsl2561_init(&i2c);
		//wait 450ms to let the sensor start reading the light
		usleep(450000);

		// initialize the Nexys4 driver
		uint32_t status = NX4IO_initialize(N4IO_BASEADDR);
		if (status != XST_SUCCESS){
			return XST_FAILURE; //results in a warning.
		}

		//Enable RGB1
		 NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);

		 //Set unsued SSEG digits to be off
		 NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, CC_BLANK);
		 NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, CC_BLANK);
	}

	configASSERT( ( xStatus == pdPASS ) );
}

