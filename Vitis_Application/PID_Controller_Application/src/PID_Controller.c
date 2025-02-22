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
 *   - `00`  Change by ±1 per button press.
 *   - `01`  Change by ±5 per button press.
 *   - `1x`  Change by ±10 per button press.
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
 */


#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "nexys4IO.h"
#include <stdlib.h>
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"


void main(){


}
