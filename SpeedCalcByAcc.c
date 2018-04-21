/*
 *  Auto-pilot robot project
 *  Module: Speed estimation by accelerometer data
 *  Author: Pedro Bertoleti
 *  Date: April/2018
 *
 *  Important notes:
 *  1) Acceleration offset values (in x and y axis) are automatically
 *     calculated in init function (init_speed_by_acc_data () )
 *  2) These module is based on DSP concepts presented on AN3397
 *     Application Note (from Freescale / NXP)
*/

/* Include section */ 
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <string.h>
#include <limits.h>
#include "SpeedCalcByAcc.h"

/* Define section */ 
#define AMOUNT_OF_ACC_DATA_FOR_CALIBRATION               (int)1000 
#define SAMPLE_RATE_ACC_HZ                               (float)200
#define RANGE_OR_SENSIBILITY_ACC                         A_FSR_2G                                                        
#define NATIVE_FLFP_FILTER_LEVEL_ACC_HZ                  ACCEL_DLPF_41                                                           
#define DISCRIMINATION_WINDOW                            (float)0.3  
#define THRESHOLD_MOVEMENT_END_CHECK                     (unsigned int)25 
#define AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER       (unsigned int)20 

/* Uncomment the line below to enable debug messages */
#define ENABLE_DEBUG_MESSAGES_SPEED_CALC

#ifdef ENABLE_DEBUG_MESSAGES_SPEED_CALC
   #define DEBUG_SPEED(...) printf(__VA_ARGS__)
#endif

/* Typedefs */
/* Every struct contains a block of variables that have similar features */
typedef struct {
    float offset_acc_x;    
    float offset_acc_y;
}TAccCalibration;

typedef struct {
    float accX_t0;
    float accY_t0;
    float accX_t1;
    float accY_t1;
}TAccSamplesXY;

typedef struct {
    float instant_speed;
    float speed_x_axis;
    float speed_y_axis;
}TInstantSpeed;

typedef struct {
    unsigned int counter_accx_zero_value;
    unsigned int counter_accy_zero_value;
}TMovementEndCheck;

typedef struct {
    unsigned int moving_average_filter_buffer_counter;
    float moving_average_filter_samples_accX[AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER];
    float moving_average_filter_samples_accY[AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER];
    void (*fill_buffer_fun_ptr)(float,float);
    float (*run_moving_average_filter_X_fun_ptr)(float);
    float (*run_moving_average_filter_Y_fun_ptr)(float);
}TMovingAverageFilter;

/* Local function prototypes section */
void  set_instant_speed(float vel);
float calculate_delta_speed(float accT0, float accT1, float sample_frequency);
void  imu_sample_ready_dmp_mode(void);
void  instant_speed_calculation(float accX_t0, float accY_t0, float accX_t1, float accY_t1);
float verify_discrimination_window(float sample);
void  refresh_movement_end_check_counter(float acc_t0, float acc_t1, unsigned int * ptrCounter);
void fill_moving_average_filter_buffer(float acc_sample_x, float acc_sample_y);
float run_moving_average_filter_X(float acc_sample_x);
float run_moving_average_filter_Y(float acc_sample_y);

/* Global variables */
static rc_imu_data_t data; 
static TAccSamplesXY AccSamplesXY;
static TAccCalibration AccCalibration;
static TInstantSpeed InstantSpeed;
static TMovingAverageFilter MovingAverageFilter;
static TMovementEndCheck MovementEndCheck;
static unsigned int imu_software_int_counter = 0;

/* Function implementation section */ 

/* Function: fill moving average filter buffer
 * Params: acceleration samples
 * Return: none
 */
void fill_moving_average_filter_buffer(float acc_sample_x, float acc_sample_y)
{
    MovingAverageFilter.moving_average_filter_samples_accX[MovingAverageFilter.moving_average_filter_buffer_counter] = acc_sample_x;
	MovingAverageFilter.moving_average_filter_samples_accY[MovingAverageFilter.moving_average_filter_buffer_counter] = acc_sample_y;
	MovingAverageFilter.moving_average_filter_buffer_counter++;
}

/* Function: run moving average filter (X-axis)
 * Params: X-axis acceleration sample
 * Return: filtered sample
 */
float run_moving_average_filter_X(float acc_sample_x)
{
	float accx_values_sum = 0.0;
	float accX_filtered = 0.0;
    int i;

    for(i=1; i<AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER; i++)
	{
		MovingAverageFilter.moving_average_filter_samples_accX[i-1] = MovingAverageFilter.moving_average_filter_samples_accX[i];		
		accx_values_sum = accx_values_sum + MovingAverageFilter.moving_average_filter_samples_accX[i];		
	}

	MovingAverageFilter.moving_average_filter_samples_accX[AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER-1] = acc_sample_x;
	accx_values_sum = accx_values_sum + MovingAverageFilter.moving_average_filter_samples_accX[AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER-1];
	accX_filtered = accx_values_sum / AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER;
	return accX_filtered;
}


/* Function: run moving average filter (Y-axis)
 * Params: Y-axis acceleration sample
 * Return: filtered sample
 */
float run_moving_average_filter_Y(float acc_sample_y)
{
	float accy_values_sum = 0.0;
	float accY_filtered = 0.0;
    int i;

    for(i=1; i<AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER; i++)
	{
		MovingAverageFilter.moving_average_filter_samples_accY[i-1] = MovingAverageFilter.moving_average_filter_samples_accY[i];		
		accy_values_sum = accy_values_sum + MovingAverageFilter.moving_average_filter_samples_accY[i];		
	}

	MovingAverageFilter.moving_average_filter_samples_accY[AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER-1] = acc_sample_y;
	accy_values_sum = accy_values_sum + MovingAverageFilter.moving_average_filter_samples_accY[AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER-1];
	accY_filtered = accy_values_sum / AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER;
	return accY_filtered;
}

/* Function: IMU sample ready callback (DMP mode)
 * Params: none
 * Return: none
 */
void imu_sample_ready_dmp_mode(void)
{
    float accX_sample;
	float accY_sample;
	float accX_filtered;
	float accY_filtered;
	
    /*  Verify if the moving average filter buffer must be filled.
        - If yes: just add current acceleration measure to buffer
        - If no:  add current acceleration measure to buffer and calculate its mean
    */	
	if (MovingAverageFilter.moving_average_filter_buffer_counter < AMOUNT_OF_SAMPLES_TO_MOVING_AVERAGE_FILTER)
	{
		/* moving average filter buffer is being filled */
        accX_sample = verify_discrimination_window(data.accel[0] - AccCalibration.offset_acc_x);
        accY_sample = verify_discrimination_window(data.accel[1] - AccCalibration.offset_acc_y);
		(*MovingAverageFilter.fill_buffer_fun_ptr)(accX_sample, accY_sample);
		return;
	}
	else
	{
		/* add current acceleration measure to buffer and calculate its mean */
        accX_sample = verify_discrimination_window(data.accel[0] - AccCalibration.offset_acc_x);
        accY_sample = verify_discrimination_window(data.accel[1] - AccCalibration.offset_acc_y);
        accX_filtered = (*MovingAverageFilter.run_moving_average_filter_X_fun_ptr)(accX_sample);
        accY_filtered = (*MovingAverageFilter.run_moving_average_filter_X_fun_ptr)(accY_sample);
	}

	imu_software_int_counter++;

    /* On every two IMU samples, speed must be calculated */
    if (imu_software_int_counter == 2)
    {
        AccSamplesXY.accX_t1 = accX_filtered;
        AccSamplesXY.accY_t1 = accY_filtered;
        instant_speed_calculation(AccSamplesXY.accX_t0, AccSamplesXY.accY_t0, AccSamplesXY.accX_t1, AccSamplesXY.accY_t1);
        imu_software_int_counter=0;
    }
    else
    {
	    AccSamplesXY.accX_t0 = accX_filtered;
        AccSamplesXY.accY_t0 = accY_filtered;
    }
}

/* Function: discrimination window step
 * Params: acceleration sample
 * Return: 0.0: acceleration sample fits discrimination window
 *         sample: acceleration sample doesn't fit discrimination window
 */
float verify_discrimination_window(float sample)
{
    if ( abs(sample) <= DISCRIMINATION_WINDOW )
        return 0.0;
    else
        return sample;
}

/* Function: refreshes movement end check counter
 * Params: accelerations (of one of the axis) in two consecutive times 
 * Return: none
*/
void refresh_movement_end_check_counter(float acc_t0, float acc_t1, unsigned int * ptrCounter)
{
	if (ptrCounter == NULL)
		return;

	if ( (acc_t0 == 0.0) && (acc_t1 == 0.0) )
    {
		*ptrCounter = (*ptrCounter) + 1;

        /* Ensure counter doesn't reaches its variable type limit (UINT_MAX) 
           If this counter value is reached, it's restored to the value defined in THRESHOLD_MOVEMENT_END_CHECK
        */
        if (*ptrCounter == UINT_MAX)
            *ptrCounter = THRESHOLD_MOVEMENT_END_CHECK;
    }
	else
		*ptrCounter = 0;
}

/* Function: calculate instant speed (based on accelerometer samples)
 * Params: none
 * Return: X and Y axis accelerations (in two consecutives times)
*/
void instant_speed_calculation(float accX_t0, float accY_t0, float accX_t1, float accY_t1)
{
	float instant_speed_x_axis, instant_speed_y_axis, absolute_speed_result;
	
	/* Do the "movement end" verification */
	refresh_movement_end_check_counter(accX_t0, accX_t1, &MovementEndCheck.counter_accx_zero_value);
	refresh_movement_end_check_counter(accY_t0, accY_t1, &MovementEndCheck.counter_accy_zero_value);

    /* Speed calculation, considering "movement end check" result */
    instant_speed_x_axis = 0.0;
    if (MovementEndCheck.counter_accx_zero_value < THRESHOLD_MOVEMENT_END_CHECK)
		instant_speed_x_axis = InstantSpeed.speed_x_axis + calculate_delta_speed(accX_t0, accX_t1, SAMPLE_RATE_ACC_HZ);

    instant_speed_y_axis = 0.0;
	if (MovementEndCheck.counter_accy_zero_value < THRESHOLD_MOVEMENT_END_CHECK)
		instant_speed_y_axis = InstantSpeed.speed_y_axis + calculate_delta_speed(accY_t0, accY_t1, SAMPLE_RATE_ACC_HZ);	

	/* Update variables (used in sucessive integration calculations) */
	InstantSpeed.speed_x_axis = instant_speed_x_axis;
	InstantSpeed.speed_y_axis = instant_speed_y_axis;

	/* Calculate robot absolute result speed */
	absolute_speed_result = (float)sqrt( (instant_speed_x_axis*instant_speed_x_axis) + (instant_speed_y_axis*instant_speed_y_axis) );
	set_instant_speed(absolute_speed_result);
}

/* Function: init speed calculation (based on acceletrometer data) 
 *           Important note: this function is assuming that robot's initial speed (in X and Y axis) is zero.
 * Params: none
 * Return: init status
 */
TStatusExec init_speed_by_acc_data(void)
{
    rc_imu_config_t conf;
    float accx_sum, accy_sum;
    int i;

    /* Global variables & function pointers init */ 
    AccSamplesXY.accX_t0 = 0.0;
    AccSamplesXY.accX_t1 = 0.0;
    AccSamplesXY.accY_t0 = 0.0;
    AccSamplesXY.accY_t1 = 0.0;
    
    InstantSpeed.instant_speed = 0.0;
    InstantSpeed.speed_x_axis = 0.0;
    InstantSpeed.speed_y_axis = 0.0;

    MovementEndCheck.counter_accx_zero_value = 0;
    MovementEndCheck.counter_accx_zero_value = 0;

    MovingAverageFilter.moving_average_filter_buffer_counter = 0;
    MovingAverageFilter.fill_buffer_fun_ptr = &fill_moving_average_filter_buffer;
    MovingAverageFilter.run_moving_average_filter_X_fun_ptr = &run_moving_average_filter_X;
    MovingAverageFilter.run_moving_average_filter_Y_fun_ptr = &run_moving_average_filter_Y;

    /* IMU init */
    if(rc_initialize())
	   return eStatusExecFailRootPrivillege;

    imu_software_int_counter = 0;    
    conf =                     rc_default_imu_config();
    conf.enable_magnetometer = 0;
    conf.accel_fsr =           RANGE_OR_SENSIBILITY_ACC;
    conf.accel_dlpf =          NATIVE_FLFP_FILTER_LEVEL_ACC_HZ;
    conf.dmp_sample_rate =     SAMPLE_RATE_ACC_HZ;

    /* Init IMU in polling mode / normal mode FOR ACCELEROMETER CALIBRATION ONLY */
    rc_initialize_imu(&data, conf);

    /* Accelerometer calibration routine  - start */
    i=0;
    accx_sum = 0.0;
    accy_sum = 0.0;
    AccCalibration.offset_acc_x = 0.0;
    AccCalibration.offset_acc_y = 0.0;

    while(i < AMOUNT_OF_ACC_DATA_FOR_CALIBRATION)
    {
        if (rc_read_accel_data(&data) < 0)
	    {
	        DEBUG_SPEED("\r\n[ERRO] Falha da leitura da IMU (leitura %d) durante a calibracao.\n", i);
    	    accx_sum = 0.0;
            accy_sum = 0.0;
	        i=0;
	        continue;
	    }

        accx_sum = accx_sum + data.accel[0];
        accy_sum = accy_sum + data.accel[1];
        i++;
    }

    AccCalibration.offset_acc_x = accx_sum / AMOUNT_OF_ACC_DATA_FOR_CALIBRATION;
    AccCalibration.offset_acc_y = accy_sum / AMOUNT_OF_ACC_DATA_FOR_CALIBRATION;
    
    DEBUG_SPEED("\r\nAccelerometer calibration:\r\n[OFFSET X] %f\r\n[OFFSET Y] %f\n", AccCalibration.offset_acc_x, AccCalibration.offset_acc_y);

    /* Accelerometer calibration routine  - end */

    /* After accelerometer calibration, IMU is init in DMP mode. It ensures constant sampling time */
    if (rc_initialize_imu_dmp(&data, conf))
	   return eStatusExecFailInitIMU;
    
    rc_set_imu_interrupt_func(imu_sample_ready_dmp_mode);
    return eStatusExecOK;
}

/* Function: end speed calculation based on accelerometer data
 * Params: none
 * Return: none
 */
void end_speed_calculation_by_acc(void)
{
    rc_stop_imu_interrupt_func();
    rc_power_off_imu();
    rc_cleanup();
}

/* Function: set instant speed
 * Params: instant speed
 * Return: none
 */
void set_instant_speed(float vel)
{
    InstantSpeed.instant_speed = vel;
}


/* Function: get instant speed (in the desired unit)
 * Params: KM_PER_HOUR: km/h speed unit
 *         M_PER_SEC: m/s speed uint
* Return: instant speed
*/
float get_instant_speed(char speed_unit)
{
    float speed_ret;

    switch(speed_unit)
    {
        case M_PER_SEC:
            speed_ret = InstantSpeed.instant_speed;
            break;

        case KM_PER_HOUR:
            speed_ret = InstantSpeed.instant_speed*3.6;
            break;

        default:
            /* Unknown speed unit. There's nothing to do here...*/
            speed_ret=0.0;
            break;
    }

    return speed_ret; 
}

/* Function: calculate speed delta (or speed variation), based on aceleration samples
 * Params: accelerations (m/s^2) and sample frequency (Hz)
 * Return: speed delta (or speed variation)
*/
float calculate_delta_speed(float accT0, float accT1, float sample_frequency)
{
	float delta = 0.0;
	float sample_period_sec = 1/sample_frequency;

    /*
	Situation #1: acceleration samples in t0 and t1 are equal. 
	Between these two acceleration samples, the inclination of acceleration curve (related to X-axis) is zero. 
	This means: 
	
	 accT0     accT1  
	   *-------*
	   |       |                     delta_speed = Acceleration graph area = T * accT0 
	   |       |                                                        or   T * accT1
	   |       |
	   |       |
	   |       |
	---------------> t(s)
	   |-  T  -|
	
	 T = sample period
	*/
	if (accT0 == accT1)
	{
		delta = sample_period_sec * accT0;
		return delta;
	}

	/*
	Situation #2: acceleration in t0 instant is greater than acceleration in t1 instant.
	Between these two acceleration samples, the inclination of acceleration curve (related to X-axis) is negative. 
	Therefore, delta speed is negative too. 
    This means: 
	
	  accT0  
	   ^   
	   | \                          delta_speed = Acceleration graph area = (-1)*(accT0 + accT1)*T*0.5     
	   |  \
  	   |   \  accT1
	   |    \ ^  
	   |       |
	---------------> t(s)
	   |-  T  -|
	
	  T = sample period
    */
	if (accT0 > accT1)
	{
		delta = (-1)*(accT0 + accT1)*sample_period_sec*0.5;
		return delta;
	}
	
	/*
	Situation #3: acceleration in t0 instant is lower than acceleration in t1 instant.
	Between these two acceleration samples, the inclination of acceleration curve (related to X-axis) is positive
	Therefore, delta speed is positive too.
	This means:

	         accT1
	           ^
	         / |                     delta_speed = Acceleration graph area = (accT0 + accT1)*T*0.5     
	       /   |
	accT0 /    |
	    ^/     |
	   |       |
	---------------> t(s)
	   |-  T  -|

	  T = sample period
    */	
	delta = (accT0 + accT1)*sample_period_sec*0.5;
        return delta;
}