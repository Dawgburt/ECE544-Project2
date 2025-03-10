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
 * @section Hardware Configuration
 * The Nexys A7 board is used for real-time PID control, with user inputs
 * from slide switches and buttons.
 *
 * @subsection Switch Configuration
 * The slide switches on the Nexys A7 board are used to adjust the setpoint and
 * tune the PID control parameters (Kp, Ki, Kd).
 *
 * - **Switches [7:6]**: Select the PID parameter to modify.
 *   - `01`  Adjust **Kp** (Proportional Gain).
 *   - `10`  Adjust **Ki** (Integral Gain).
 *   - `11`  Adjust **Kd** (Derivative Gain).
 *
 * - **Switches [5:4]**: Control the step size for increments/decrements.
 *   - `00`  Change by 1 per button press.
 *   - `01`  Change by 5 per button press.
 *   - `1x`  Change by 10 per button press.
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
 * @subsection Button Configuration
 * - **BtnU (Up Button)**: Increments the selected parameter.
 * - **BtnD (Down Button)**: Decrements the selected parameter.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author Nick A (nick.allmeyer@pdx.edu)
 * @date 2025-02-21
 *
 * @notes
 * 21-FEB-2025  PN  Started implementation of PID controller for LED brightness control.
 *                 	ChatGPT assisted with debugging build issues and making templates for functions.
 * 24-FEB-2025  PN  Updated vInputTask with a case statement for switch and button functionality.
 * 25-FEB-2025  PN  Implemented anti-windup in PID controller, ensured PWM scales correctly.
 *                 	Adjusted sensor reading frequency for improved control response.
 * 1-MAR-2025	PN	Tuning complete. Debugging complete. Works as expected but is not "perfect"
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

// Nexys4IO Peripheral Definitions
#define N4IO_DEVICE_ID          XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR           XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR           XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

//PID Default Values
#define DEFAULT_Kp 4.8828125
#define DEFAULT_Ki 0.244140625
#define DEFAULT_Kd 6.103515625	//Reducing makes LED transitions more visible

// Peripheral Instances
XIic i2c;
XGpio btns, switches, pwm;

// Task Handles
TaskHandle_t xSensorTask, xPIDTask, xDisplayTask, xInputTask;

// Global Variables
float DEFAULT_lux = 50;
float target_lux = 50;
float current_lux;    // Sensor reading
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
void PrintToLEDs(float setpoint, float current_lux);

// === MAIN FUNCTION ===
 int main() {
    xil_printf("Starting FreeRTOS PID Control Project...\r\n");

    // Initialize Nexys4IO
    int status;

    status = NX4IO_initialize(N4IO_BASEADDR);
    if (status == XST_SUCCESS) {
        xil_printf("NX4IO successfully initialized.\r\n");
    } else {
        xil_printf("ERROR: NX4IO initialization failed! Status code: %d\r\n", status);
        return XST_FAILURE;  // Exit the function or handle error accordingly
    }

    // Enable only the BLUE channel of RGB1
    NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);

    // Initialize I2C (XIic)
    status = XIic_Initialize(&i2c, XPAR_IIC_0_DEVICE_ID);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: I2C Initialization Failed!\r\n");
        return XST_FAILURE;
    }

    // Set I2C options for repeated start and master mode
    XIic_SetOptions(&i2c, XII_REPEATED_START_OPTION | XII_SEND_10_BIT_OPTION);

    // Start the I2C driver
    status = XIic_Start(&i2c);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: I2C Start Failed!\r\n");
        return XST_FAILURE;
    }

    xil_printf("I2C Initialized Successfully\r\n");

    // Initialize Peripherals
    tsl2561_init(&i2c);


    status = XGpio_Initialize(&btns, XPAR_AXI_GPIO_1_DEVICE_ID);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: GPIO (Buttons) Initialization Failed!\r\n");
        return XST_FAILURE;
    }

    status = XGpio_Initialize(&switches, XPAR_AXI_GPIO_1_DEVICE_ID);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: GPIO (Switches) Initialization Failed!\r\n");
        return XST_FAILURE;
    }

    XGpio_InterruptEnable(&btns, XGPIO_IR_CH1_MASK);

    xil_printf("All Peripherals Initialized Successfully\r\n");

    // Initialize PID
    pid_init(&pid, DEFAULT_Kp, DEFAULT_Ki, DEFAULT_Kd);  // Default Kp, Ki, Kd values

#if _DEBUG
    xil_printf("DEBUG main(): PID Controller Initialized - Kp: %d, Ki: %d, Kd: %d, Target Lux: %d\r\n",
                (int)(pid.Kp * 100), (int)(pid.Ki * 100), (int)(pid.Kd * 100), (int)(target_lux));
    xil_printf("Free Heap Size: %d bytes\n", xPortGetFreeHeapSize());
#endif

    // Create Queue
    xLuxQueue = xQueueCreate(5, sizeof(float));

    // Create FreeRTOS Tasks
    xTaskCreate(vSensorTask, "SensorTask", 512, NULL, 2, &xSensorTask);
    xTaskCreate(vPIDTask, "PIDTask", 768, NULL, 3, &xPIDTask);
    xTaskCreate(vDisplayTask, "DisplayTask", 512, NULL, 1, &xDisplayTask);
    xTaskCreate(vInputTask, "InputTask", 768, NULL, 3, &xInputTask);

    // Start Scheduler
    vTaskStartScheduler();

    while (1); // Should never reach here
}

 /**
  * @brief FreeRTOS task for reading sensor data and updating luminosity values.
  *
  * @param pvParameters Unused parameter for FreeRTOS task compatibility.
  */
void vSensorTask(void *pvParameters) {
    while (1) {
        current_lux = tsl2561_readLux(&i2c);

#if _DEBUG
    xil_printf("DEBUG vSensorTask: Current_Lux(scaled by 100): %d\r\n",
                (int)(current_lux * 100));
#endif

        xQueueSend(xLuxQueue, &current_lux, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/**
 * @brief FreeRTOS task for computing PID output and adjusting LED brightness.
 *
 * @param pvParameters Unused parameter for FreeRTOS task compatibility.
 */
void vPIDTask(void *pvParameters) {
    float lux_input;
    while (1) {
        if (xQueueReceive(xLuxQueue, &current_lux, portMAX_DELAY)) {
             pwm_duty_cycle = pid_compute(&pid, target_lux, current_lux);

             // Convert PID output to 0-100% range
             int pwm_percentage = (int)((pwm_duty_cycle / 255.0) * 100);
             if (pwm_percentage > 100) pwm_percentage = 100;
             if (pwm_percentage < 0) pwm_percentage = 0;

             // Force apply the new PWM value
             NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwm_percentage);

#if _DEBUG
    xil_printf("DEBUG vPIDTask: Setpoint: %d | Current Lux: %d | PID Output: %d | PWM: %d%%\r\n",
            	(int)target_lux, (int)(current_lux*100), (int)pwm_duty_cycle, (int)((pwm_duty_cycle / 255.0) * 100));
#endif

        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}


/**
 * @brief FreeRTOS task for displaying values to the 7-segment display.
 *
 * @param pvParameters Unused parameter for FreeRTOS task compatibility.
 */
void vDisplayTask(void *pvParameters) {
    while (1) {

    	PrintToLEDs(target_lux, current_lux);
#if _DEBUG
    xil_printf("DEBUG vDisplayTask: Lux: %d | Target: %d | pid_output: %d\r\n",
                (int)(current_lux * 100), (int)(target_lux), (int)(pwm_duty_cycle));
#endif
        vTaskDelay(pdMS_TO_TICKS(500)); // Update every second
    }
}

/**
 * @brief FreeRTOS task for handling user input via switches and buttons.
 *
 * @param pvParameters Unused parameter for FreeRTOS task compatibility.
 */
void vInputTask(void *pvParameters) {
    static float prev_Kp = 1.0, prev_Ki = 0.1, prev_Kd = 0.05;  // Stores last known values
    static bool was_Kp_disabled = false, was_Ki_disabled = false, was_Kd_disabled = false;  // Track previous state

    while (1) {
        int btn_state = XGpio_DiscreteRead(&btns, 1);
        int sw_state = XGpio_DiscreteRead(&switches, 2);
        float *param_to_adjust = NULL;

        // Setpoint adjustment (Switch[3])
        if (sw_state & SW4) {
            param_to_adjust = &target_lux;
        } else {
            // Select PID parameter to adjust (Switches [7:6])
            switch ((sw_state >> 6) & 0x3) { // Extract bits [7:6]
                case 0b01: param_to_adjust = &pid.Kp; break; // Adjust Kp
                case 0b10: param_to_adjust = &pid.Ki; break; // Adjust Ki
                case 0b11: param_to_adjust = &pid.Kd; break; // Adjust Kd
            }
        }

        // Determine step size based on Switches [5:4]
        switch ((sw_state >> 4) & 0x3) { // Extract bits [5:4]
            case 0b00: step_size = 1.0; break;
            case 0b01: step_size = 5.0; break;
            case 0b10: // Falls through
            case 0b11: step_size = 10.0; break;
        }

        // Scale step size for Ki and Kd
        if (param_to_adjust == &pid.Ki) {
            step_size *= 0.1;
        }

        else if (param_to_adjust == &pid.Kd) {
            step_size *= 0.05;
        }

        // Enable/Disable PID control terms based on Switches [2:0]
        if (sw_state & SW1) {
            if (was_Kp_disabled) {  // Restore only if it was previously disabled
                pid.Kp = prev_Kp;
                was_Kp_disabled = false;
            }
        } else {
            if (!was_Kp_disabled) {  // Save only once when switching off
                prev_Kp = pid.Kp;
                pid.Kp = 0.0;
                was_Kp_disabled = true;
            }
        }

        if (sw_state & SW2) {
            if (was_Ki_disabled) {  // Restore only if it was previously disabled
                pid.Ki = prev_Ki;
                was_Ki_disabled = false;
            }
        } else {
            if (!was_Ki_disabled) {  // Save only once when switching off
                prev_Ki = pid.Ki;
                pid.Ki = 0.0;
                was_Ki_disabled = true;
            }
        }

        if (sw_state & SW3) {
            if (was_Kd_disabled) {  // Restore only if it was previously disabled
                pid.Kd = prev_Kd;
                was_Kd_disabled = false;
            }
        } else {
            if (!was_Kd_disabled) {  // Save only once when switching off
                prev_Kd = pid.Kd;
                pid.Kd = 0.0;
                was_Kd_disabled = true;
            }
        }

        // Adjust selected parameter
        if (param_to_adjust) {
            if (btn_state & 0x08) *param_to_adjust += step_size; //BtnU
            if (btn_state & 0x04) *param_to_adjust -= step_size; //BtnD
        }

       if (btn_state & 0x10) { pid.Kp = DEFAULT_Kp; pid.Ki = DEFAULT_Ki; pid.Kd = DEFAULT_Kd; } //BtnC


       if (btn_state & 0x01) {
           target_lux = DEFAULT_lux;      // Reset setpoint to 100
           pid.integral = 0.0;     // Reset integral term to prevent windup
           pid.prev_error = 0.0;   // Reset previous error to prevent jumps
           xil_printf("PID Reset: Setpoint = %d, Integral = 0, Prev Error = 0\r\n", (int)target_lux);
       }


#if _DEBUG
    xil_printf("DEBUG vInputTask: Setpoint: %d | Kp: %d | Ki: %d | Kd: %d | btn_state: %d\r\n",
               (int)(target_lux), (int)(pid.Kp * 100), (int)(pid.Ki * 100), (int)(pid.Kd * 100), btn_state);
#endif

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// === PRINT TO LEDS FUNCTION ===
void PrintToLEDs(float setpoint, float current_lux) {
    int setpoint_scaled = (int)(setpoint);      // Scale setpoint for display

    int lux_scaled_up = (int)(current_lux * 1000);        // Scale current lux for display
    int lux_scaled = (int)((lux_scaled_up / 255.0) * 100);
    lux_scaled = lux_scaled / 2;
    if (lux_scaled > 255) lux_scaled = 255;
    if (lux_scaled < 0) lux_scaled = 0;

    // Extract individual digits for setpoint
    int setpoint_ones = setpoint_scaled % 10;
    int setpoint_tens = (setpoint_scaled / 10) % 10;
    int setpoint_hundreds = (setpoint_scaled / 100) % 10;

    // Extract individual digits for current lux
    int lux_ones = lux_scaled % 10;
    int lux_tens = (lux_scaled / 10) % 10;
    int lux_hundreds = (lux_scaled / 100) % 10;

    // Setpoint: Digits[7:5]
    NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, (enum _NX4IO_charcodes)setpoint_hundreds); // Hundreds place
    NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, (enum _NX4IO_charcodes)setpoint_tens);     // Tens place
    NX4IO_SSEG_setDigit(SSEGHI, DIGIT5, (enum _NX4IO_charcodes)setpoint_ones);     // Ones place

    // Blank Digit[4]
    NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, CC_BLANK);

    // Current Lux: Digits[3:1]
    NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, (enum _NX4IO_charcodes)lux_hundreds); // Hundreds place
    NX4IO_SSEG_setDigit(SSEGLO, DIGIT2, (enum _NX4IO_charcodes)lux_tens);     // Tens place
    NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, (enum _NX4IO_charcodes)lux_ones);     // Ones place

    // Blank Digit[0]
    NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, CC_BLANK);
}


