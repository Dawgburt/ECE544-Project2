/*
 * pid.h
 *
 *  Created on: Feb 24, 2025
 *      Author: pnevi
 */

#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
    float prev_error, integral;
} PIDController;

void pid_init(PIDController *pid, float Kp, float Ki, float Kd);
float pid_compute(PIDController *pid, float setpoint, float measured);

#endif
