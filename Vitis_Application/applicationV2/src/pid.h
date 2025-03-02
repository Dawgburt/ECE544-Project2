#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    unsigned long prev_time; // Added to track previous timestamp
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd);
float pid_compute(PIDController *pid, float setpoint, float measured);

#endif // PID_H
