/**
 * @file PID_Controller.c
 * @brief Implementation of the PID controller for luminosity control using FreeRTOS.
 *
 * This file contains the function definitions for implementing a PID controller
 * to regulate LED brightness based on the TSL2561 luminosity sensor readings.
 * The PID controller dynamically adjusts the LED's PWM duty cycle to maintain
 * a target lux level.
 *
 * The controller operates in a closed-loop system using FreeRTOS tasks and
 * queues for real-time adjustments based on ambient light changes.
 *
 * @section Switch Configuration
 * The slide switches on the Nexys A7 board are used to adjust the setpoint and
 * tune the PID control parameters (Kp, Ki, Kd).
 *
 * - **Switches [7:6]**: Select the PID parameter to modify.
 *   - `01`  Adjust **Kp** (Proportional Gain).
 *   - `10`  Adjust **Ki** (Integral Gain).
 *   - `11`  Adjust **Kd** (Derivative Gain).
 *
 * - **Switches [5:4]**: Control the step size for increments/decrements.
 *   - `00`  Change by �1 per button press.
 *   - `01`  Change by �5 per button press.
 *   - `1x`  Change by �10 per button press.
 *
 * - **Switch [3]**: Selects whether the **setpoint** is being adjusted.
 *   - `1` Buttons modify the **setpoint**.
 *   - `0` No change in setpoint on button press.
 *
 * - **Switches [2:0]**: Enable/disable PID components for experimentation.
 *   - **Switch [2]** (Derivative Control):
 *     - `0`  Disable D control.
 *     - `1`  Enable D control.
 *   - **Switch [1]** (Integral Control):
 *     - `0`  Disable I control.
 *     - `1`  Enable I control.
 *   - **Switch [0]** (Proportional Control):
 *     - `0`  Disable P control.
 *     - `1`  Enable P control.
 *
 * @section Button Configuration
 * - **BtnU (Up Button)**: Increments the selected parameter.
 * - **BtnD (Down Button)**: Decrements the selected parameter.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author [Partner's Name]
 * @date 2025-02-21
 *
 * @notes
 * 21-FEB-2025  PN  Started implementation of PID controller for LED brightness control.
 * 					ChatGPT assisted with debugging build issues and making templates for functions
 * 24-FEB-2025	PN	Updated vInputTask with a case statement for switch and button functionality
 */


#include <stdio.h>
#include "xparameters.h"
#include "xiic.h"
#include "xgpio.h"
#include "xil_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <math.h>
#include "tsl2561.h"  // Custom TSL2561 driver
#include "pid.h"      // Custom PID controller
#include "xuartlite.h"
#include "nexys4io.h"

#define _DEBUG 1
// Define switch bit positions
#define SW1  (1 << 0)
#define SW2  (1 << 1)
#define SW3  (1 << 2)
#define SW4  (1 << 3)
#define SW5  (1 << 4)
#define SW6  (1 << 5)
#define SW7  (1 << 6)
#define SW8  (1 << 7)
#define SW9  (1 << 8)
#define SW10 (1 << 9)
#define SW11 (1 << 10)
#define SW12 (1 << 11)
#define SW13 (1 << 12)
#define SW14 (1 << 13)
#define SW15 (1 << 14)
#define SW16 (1 << 15)

// Peripheral Instances
XIic i2c;
XGpio btns, switches, pwm;

// Task Handles
TaskHandle_t xSensorTask, xPIDTask, xDisplayTask, xInputTask;

// Global Variables
float target_lux = 100.0;
float current_lux = 0.0;    // Sensor reading
float pwm_duty_cycle = 0.5; // PWM Duty cycle (0.0 - 1.0)
float step_size = 1.0;

// PID Controller
PIDController pid;

// FreeRTOS Queue
QueueHandle_t xLuxQueue;

// Function Prototypes
void vSensorTask(void *pvParameters);
void vPIDTask(void *pvParameters);
void vDisplayTask(void *pvParameters);
void vInputTask(void *pvParameters);
void init_uart();

// === MAIN FUNCTION ===
 int main() {
    xil_printf("Starting FreeRTOS PID Control Project...\r\n");

    // Initialize UART for debugging
    init_uart();

    // Initialize I2C (XIic)
    int status = XIic_Initialize(&i2c, XPAR_IIC_0_DEVICE_ID);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: I2C Initialization Failed!\r\n");
        return -1;
    }

    // Set I2C options for repeated start and master mode
    XIic_SetOptions(&i2c, XII_REPEATED_START_OPTION | XII_SEND_10_BIT_OPTION);

    // Start the I2C driver
    XIic_Start(&i2c);

    xil_printf("I2C Initialized Successfully\r\n");

    // Initialize Peripherals
    tsl2561_init(&i2c);
    XGpio_Initialize(&btns, XPAR_AXI_GPIO_1_DEVICE_ID);
    XGpio_Initialize(&switches, XPAR_AXI_GPIO_1_DEVICE_ID);
	XGpio_InterruptEnable( &btns, XGPIO_IR_CH1_MASK );
    //XGpio_Initialize(&pwm, XPAR_AXI_GPIO_1_DEVICE_ID);


    // Initialize PID
    pid_init(&pid, 1.0, 0.1, 0.05);  // Default Kp, Ki, Kd values

#if _DEBUG
    xil_printf("DEBUG main(): PID Controller Initialized - Kp: %d, Ki: %d, Kd: %d, Target Lux: %d\r\n",
                (int)(pid.Kp * 100), (int)(pid.Ki * 100), (int)(pid.Kd * 100), (int)(target_lux));
#endif


    // Create Queue
    xLuxQueue = xQueueCreate(5, sizeof(float));  // Reduce queue size to 3 elements


    // Create FreeRTOS Tasks
    xTaskCreate(vSensorTask, "SensorTask", 1024, NULL, 2, &xSensorTask);
    xTaskCreate(vPIDTask, "PIDTask", 1024, NULL, 3, &xPIDTask);
    xTaskCreate(vDisplayTask, "DisplayTask", 512, NULL, 1, &xDisplayTask);
    xTaskCreate(vInputTask, "InputTask", 1024, NULL, 2, &xInputTask);


    // Start Scheduler
    vTaskStartScheduler();

    while (1); // Should never reach here
}

// === SENSOR TASK ===
void vSensorTask(void *pvParameters) {
    while (1) {
        current_lux = tsl2561_readLux(&i2c);
        xQueueSend(xLuxQueue, &current_lux, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500)); // Read every 500ms
    }
}

// === PID CONTROL TASK ===
void vPIDTask(void *pvParameters) {
    float lux_input;
    while (1) {
        if (xQueueReceive(xLuxQueue, &lux_input, portMAX_DELAY)) {
            pwm_duty_cycle = pid_compute(&pid, target_lux, lux_input);
           // XGpio_DiscreteWrite(&pwm, 2, (int)(pwm_duty_cycle * 255));
            NX4IO_RGBLED_setRGB_DATA(2, 0x000000FF);  // Force max brightness on BLUE channel

#if _DEBUG
    xil_printf("DEBUG vPIDTask: Lux(scaled by 100): %d | Target: %d | PWM(scaled by 100): %d\r\n",
                (int)(lux_input * 100), (int)(target_lux), (int)(pwm_duty_cycle * 100));
#endif

        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}


// === DISPLAY TASK ===
void vDisplayTask(void *pvParameters) {
    while (1) {
#if _DEBUG
    xil_printf("DEBUG vDisplayTask: Lux: %d | Target: %d | PWM: %d\r\n",
                (int)(current_lux * 100), (int)(target_lux), (int)(pwm_duty_cycle * 100));
#endif
        vTaskDelay(pdMS_TO_TICKS(500)); // Update every second
    }
}


void vInputTask(void *pvParameters) {
    while (1) {
        int btn_state = XGpio_DiscreteRead(&btns, 1);
        int sw_state = XGpio_DiscreteRead(&switches, 2);

        //float step_size = 1.0;
        float *param_to_adjust = NULL;

//#if _DEBUG
 //    xil_printf("DEBUG vInputTask: Switch State: %d | Button State: %d | step_size: %d\r\n", sw_state, btn_state,
 //    		(int)step_size);
// #endif


     	//if (!(sw_state & SW1)) pid.Kp = 0;
     	//if (sw_state & SW1) pid.Kp = prev_Kp;
     	//if (sw_state & SW2)
     	//if (sw_state & SW3)

     	if (sw_state & SW4) 			param_to_adjust = &target_lux;

     	if (!(sw_state & (SW5 | SW6))) 	step_size = 1.0;
        if (sw_state & SW5) 			step_size = 5.0;
        if (sw_state & SW6) 			step_size = 10.0;

        if (sw_state & SW7) 						param_to_adjust = &pid.Kp;
        if (sw_state & SW8) 						param_to_adjust = &pid.Ki;
        if ((sw_state & SW7) && (sw_state & SW8)) 	param_to_adjust = &pid.Kd;


        if (param_to_adjust) {
            if (btn_state & 0x08) *param_to_adjust += step_size; //Button UP
            if (btn_state & 0x04) *param_to_adjust -= step_size; //Button Down
        }

        #if _DEBUG
            xil_printf("DEBUG vInputTask: Setpoint: %d | Kp: %d | Ki: %d | Kd: %d\r\n",
                        (int)(target_lux * 100), (int)(pid.Kp * 100), (int)(pid.Ki * 100), (int)(pid.Kd * 100));
        #endif

        vTaskDelay(pdMS_TO_TICKS(500));  // **Increased delay from 250ms to 500ms**
    }
}




XUartLite UartLite;
#define UART_DEVICE_ID  XPAR_UARTLITE_0_DEVICE_ID

void init_uart() {
    int status = XUartLite_Initialize(&UartLite, UART_DEVICE_ID);
    if (status != XST_SUCCESS) {
        xil_printf("UART Init Failed\r\n");
    }
}
