/*
 *  Auto-pilot robot project
 *  Module: PID Control
 *  Author: Pedro Bertoleti
 *  Date: April/2018
*/

/* Include section */ 
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "PIDControl.h"

/* Define section */ 
#define KP_CONSTANT     (float)1.0
#define KI_CONSTANT     (float)0.5
#define KD_CONSTANT     (float)0.0
#define PID_MAX_LIMIT   (float)1.0  /* 100% */
#define PID_MIN_LIMIT   (float)0.0  /* 0%   */

/* Global variables */
/* Every struct contains a block of variables that have similar features */
typedef struct {
    float setpoint_pid;
    float last_error;
    float integral_acum;
}TPIDController;

static TPIDController PIDController;

/* Function implementation section */ 

/* Function: setpoint configuration and related inits
 * Params: setpoint
 * Return: none
*/
void setpoint_config(float setpoint_value)
{   
   PIDController.last_error = 0.0;
   PIDController.integral_acum = 0.0;
   PIDController.setpoint_pid = setpoint_value;
}

/* Function: read current setpoint value
 * Params: none
 * Return: setpoint
 */
float read_current_setpoint(void)
{
   return PIDController.setpoint_pid;
}

/* Function: make PID calculation
 * Params: process variable and time_step
 * Returno: PID final result
 */
float calc_PID(float process_variable, float time_step)
{
    float current_error;
    float p_factor;
    float i_factor;
    float f_factor;
    float pid_final_result;

    /* Current error calculation */
    current_error = PIDController.setpoint_pid - process_variable;

    /* P, I and D terms calculation */
    p_factor = KP_CONSTANT * current_error;

    i_factor = PIDController.integral_acum + (KI_CONSTANT * current_error * time_step);
    PIDController.integral_acum = i_factor;

    f_factor = KD_CONSTANT * ( (PIDController.last_error - current_error) / time_step );
    PIDController.last_error = current_error;

    /* PID value calculation */
    pid_final_result = p_factor + i_factor + f_factor;
    
    /* Ensure PID final result will fit the valid range */
    if (pid_final_result > PID_MAX_LIMIT)
      pid_final_result = PID_MAX_LIMIT;

    if (pid_final_result < PID_MIN_LIMIT)
      pid_final_result = PID_MIN_LIMIT;

    /* Return PID final result */
    return pid_final_result;
}
