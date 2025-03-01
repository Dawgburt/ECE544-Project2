/*
 * pid.c
 *
 *  Created on: Feb 24, 2025
 *      Author: pnevi
 */


#include "pid.h"


void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}



#define INTEGRAL_LIMIT 100.0   // Limit the integral term
#define OUTPUT_MIN 0.0         // Minimum duty cycle (0%)
#define OUTPUT_MAX 255.0       // Maximum duty cycle (100%)

float pid_compute(PIDController *pid, float setpoint, float measured) {
    float error = setpoint - measured;   // Difference between target and actual
    pid->integral += error * 0.01;       // Accumulate error for integral term (smaller factor for smoother changes)

    // Prevent integral windup
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;

    float derivative = (error - pid->prev_error) * 0.01; // Change in error
    pid->prev_error = error;

    // Compute PID output
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Ensure output stays within the valid range (0-255)
    if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;

    return output;  // Now properly scaled duty cycle (0-255)
}






