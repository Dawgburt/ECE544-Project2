/**
 * @file pid.h
 * @brief Header file for PID controller.
 *
 * This file provides the function prototypes, macros, and data types
 * necessary for running the PID functions in pid.c.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author Nick A (nick.allmeyer@pdx.edu)
 *
 * @date 2025-02-21
 *
 * @notes
 * Doxygen comments generated via ChatGPT.
 */

#ifndef PID_H
#define PID_H

/**
 * @struct PIDController
 * @brief Structure representing a PID controller.
 */
typedef struct {
    float Kp;          ///< Proportional gain
    float Ki;          ///< Integral gain
    float Kd;          ///< Derivative gain
    float integral;    ///< Integral term accumulation
    float prev_error;  ///< Previous error value for derivative calculation
    unsigned long prev_time; ///< Previous timestamp in milliseconds
} PIDController;

/**
 * @brief Initializes the PID controller with given gain parameters.
 *
 * @param pid Pointer to the PIDController structure.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void pid_init(PIDController *pid, float kp, float ki, float kd);

/**
 * @brief Computes the PID control output based on the setpoint and measured value.
 *
 * @param pid Pointer to the PIDController structure.
 * @param setpoint Desired target value.
 * @param measured Actual measured value.
 * @return Computed PID output.
 */
float pid_compute(PIDController *pid, float setpoint, float measured);

#endif // PID_H
