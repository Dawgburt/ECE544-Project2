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

// Peripheral Instances
XIic i2c;
XGpio btns, switches, pwm;

// Task Handles
TaskHandle_t xSensorTask, xPIDTask, xDisplayTask, xInputTask;

// Global Variables
float target_lux = 100.0;   // Default target Lux
float current_lux = 0.0;    // Sensor reading
float pwm_duty_cycle = 0.5; // PWM Duty cycle (0.0 - 1.0)

// PID Controller
PIDController pid;

// FreeRTOS Queue
QueueHandle_t xLuxQueue;

// Function Prototypes
void vSensorTask(void *pvParameters);
void vPIDTask(void *pvParameters);
void vDisplayTask(void *pvParameters);
void vInputTask(void *pvParameters);

// === MAIN FUNCTION ===
int main() {
    xil_printf("Starting FreeRTOS PID Control Project...\n");

    // Initialize Peripherals
    tsl2561_init(&i2c);
    XGpio_Initialize(&btns, XPAR_AXI_GPIO_1_DEVICE_ID);
    XGpio_Initialize(&switches, XPAR_AXI_GPIO_1_DEVICE_ID);
    XGpio_Initialize(&pwm, XPAR_AXI_GPIO_1_DEVICE_ID);


    // Initialize PID
    pid_init(&pid, 1.0, 0.1, 0.05);  // Default Kp, Ki, Kd values

    // Create Queue
    xLuxQueue = xQueueCreate(3, sizeof(float));  // Reduce queue size to 3 elements


    // Create FreeRTOS Tasks
    xTaskCreate(vSensorTask, "SensorTask", 64, NULL, 2, &xSensorTask);
    xTaskCreate(vPIDTask, "PIDTask", 64, NULL, 3, &xPIDTask);
    xTaskCreate(vDisplayTask, "DisplayTask", 32, NULL, 1, &xDisplayTask);
    xTaskCreate(vInputTask, "InputTask", 32, NULL, 2, &xInputTask);


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
            XGpio_DiscreteWrite(&pwm, 2, (int)(pwm_duty_cycle * 255));
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Run every 100ms
    }
}

// === DISPLAY TASK ===
void vDisplayTask(void *pvParameters) {
    while (1) {
        xil_printf("Lux: %.2f | Target: %.2f | PWM: %.2f\n", current_lux, target_lux, pwm_duty_cycle);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
    }
}


// === INPUT TASK (Handles Buttons & Switches) ===
void vInputTask(void *pvParameters) {
    while (1) {
        int btn_state = XGpio_DiscreteRead(&btns, 1); // Read Buttons (Channel 1)
        int sw_state = XGpio_DiscreteRead(&switches, 1); // Read Switches (Channel 1)

        float step_size = 1.0;  // Default increment size
        float *param_to_adjust = NULL; // Pointer to selected parameter

        // Determine step size based on switches [5:4]
        switch ((sw_state >> 4) & 0x3) {  // Extract bits [5:4]
            case 0b00: step_size = 1.0;  break;
            case 0b01: step_size = 5.0;  break;
            case 0b10: step_size = 10.0; break;
            case 0b11: step_size = 10.0; break;  // Any case where Switch[5] is set
        }

        // Determine which parameter is being adjusted based on switches [7:6] and [3]
        switch ((sw_state >> 6) & 0x3) {  // Extract bits [7:6]
            case 0b01: param_to_adjust = &pid.Kp; break;  // Adjust Kp
            case 0b10: param_to_adjust = &pid.Ki; break;  // Adjust Ki
            case 0b11: param_to_adjust = &pid.Kd; break;  // Adjust Kd
            default:
                if (sw_state & 0x08)  // Switch[3] controls setpoint
                    param_to_adjust = &target_lux;
                break;
        }

        // Adjust selected parameter if a button is pressed
        if (param_to_adjust) {
            if (btn_state & 0x01) *param_to_adjust += step_size; // Button Up (Increment)
            if (btn_state & 0x02) *param_to_adjust -= step_size; // Button Down (Decrement)
        }

        // Enable/Disable PID components based on switches [2:0]
        pid.Kp = (sw_state & 0x01) ? pid.Kp : 0.0;  // Switch[0] enables/disables P control
        pid.Ki = (sw_state & 0x02) ? pid.Ki : 0.0;  // Switch[1] enables/disables I control
        pid.Kd = (sw_state & 0x04) ? pid.Kd : 0.0;  // Switch[2] enables/disables D control

        xil_printf("Setpoint: %.2f | Kp: %.2f | Ki: %.2f | Kd: %.2f | Step: %.1f\n", target_lux, pid.Kp, pid.Ki, pid.Kd, step_size);

        vTaskDelay(pdMS_TO_TICKS(250)); // Debounce delay
    }
}
