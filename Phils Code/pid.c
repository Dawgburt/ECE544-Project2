/**
 * @file pid.c
 * @brief Implementation of the PID control algorithm for luminosity regulation.
 *
 * This file contains the function definitions for a Proportional-Integral-Derivative (PID) controller
 * used to regulate LED brightness based on the TSL2561 luminosity sensor readings. The PID controller
 * dynamically adjusts the LED's PWM duty cycle to maintain a target lux level by minimizing the error
 * between the desired and actual light intensity.
 *
 * The PID algorithm includes features such as integral windup prevention and scaled output adjustments
 * to ensure stability and smooth transitions in brightness control.
 *
 * @section PID Control Implementation
 * The PID controller follows the standard equation:
 *
 *      output = (Kp * error) + (Ki * integral) + (Kd * derivative)
 *
 * - **Kp (Proportional Gain)**: Determines reaction strength to the current error.
 * - **Ki (Integral Gain)**: Addresses accumulated past errors to eliminate steady-state error.
 * - **Kd (Derivative Gain)**: Predicts future error changes to reduce overshoot and oscillations.
 *
 * The controller updates periodically and applies corrections to the LED brightness via PWM control.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author Nick A (nick.allmeyer@pdx.edu)
 * @date 2025-02-21
 *
 * @notes
 * Doxygen comments generated via ChatGPT.
 */

#include "pid.h"
#include "xil_printf.h"
#include "FreeRTOS.h"
#include "task.h"

#define INTEGRAL_LIMIT 100.0   // Limit the integral term
#define OUTPUT_MIN 0.0         // Minimum duty cycle (0%)
#define OUTPUT_MAX 255.0       // Maximum duty cycle (100%)

/**
 * @brief Initializes the PID controller with given gain parameters.
 *
 * @param pid Pointer to the PIDController structure.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->prev_time = xTaskGetTickCount(); // Get initial FreeRTOS time in ticks
}

/**
 * @brief Computes the PID control output based on the setpoint and measured value.
 *
 * @param pid Pointer to the PIDController structure.
 * @param setpoint Desired target value.
 * @param measured Actual measured value.
 * @return Computed PID output.
 */
float pid_compute(PIDController *pid, float setpoint, float measured) {
    float error = setpoint - measured;

    // Get current FreeRTOS time in milliseconds
    TickType_t current_time = xTaskGetTickCount();

    // Compute dt in seconds
    float dt = (current_time - pid->prev_time) / 1000.0; // Convert ms to seconds
    if (dt <= 0.0) dt = 0.01;
    pid->prev_time = current_time;

    // Accumulate error for integral term with adaptive scaling
    pid->integral += error * dt;

    // Prevent integral windup
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;

    // Derivative term using dt
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Compute PID output
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Ensure output stays within range (0-255)
    if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;

#if _DEBUG
    xil_printf("DEBUG PID.c: Error=%.2f, Integral=%.2f, Derivative=%.2f, Output=%.2f, dt=%.5f\r\n",
               error, pid->integral, derivative, output, dt);
#endif

    return output;
}
