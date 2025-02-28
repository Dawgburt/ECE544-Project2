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



float pid_compute(PIDController *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    if (output > 1.0) output = 1.0;
    if (output < 0.0) output = 0.0;

    return output;
}
