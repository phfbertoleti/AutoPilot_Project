/*
 *  Auto-pilot robot project
 *  Module: Header of PID Control
 *  Author: Pedro Bertoleti
 *  Date: April/2018
*/

#ifndef PIDCONTROL_H_
#define PIDCONTROL_H_

/* Global function prototypes section */ 
void setpoint_config(float setpoint_value);
float read_current_setpoint(void);
float calc_PID(float process_variable, float time_step);

#endif
