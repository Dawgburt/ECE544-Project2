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



#define INTEGRAL_LIMIT 10.0  // Adjust this limit as needed

float pid_compute(PIDController *pid, float setpoint, float measured) {
    float error = setpoint - measured;

    // Prevent integral windup
    pid->integral += error;
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;

    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Ensure output is within a valid range for PWM
    if (output > 255.0) output = 255.0;
    if (output < 0.0) output = 0.0;


    return output;
}

