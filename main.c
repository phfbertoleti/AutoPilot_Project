/*
 *  Auto-pilot robot project
 *  Module: main program
 *  Author: Pedro Bertoleti
 *  Date: April/2018
*/

/* Include section */ 
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <time.h>
#include <MQTTClient.h>
#include "SpeedCalcByAcc.h"
#include "PIDControl.h"
#include "GeneralProjectParams.h"

/* Define section */ 
#define MQTT_ADDRESS                     "tcp://iot.eclipse.org"
#define CLIENTID                         "MQTTAutoPilotBBBlue"
#define MQTT_PUBLISH_TOPIC               "MQTTinstant_speedAutoPilot"
#define MQTT_AUTOPILOT_SUBSCRIBE_TOPIC   "MQTTinstant_speedAutoPilotCmd"

#define SPEED_UNIT   (int)KM_PER_HOUR

#define TIME_STEP_PID_CONTROL  (float)1  //s

/* Uncomment the line below to enable debug messages */
#define ENABLE_DEBUG_MESSAGES_MAIN

/* Uncomment the line below to enable PID data logging (in a csv file) */ 
#define ENABLE_PID_DATA_LOGGING_CSV

#ifdef ENABLE_DEBUG_MESSAGES_MAIN
   #define DEBUG_MAIN(...) printf(__VA_ARGS__)
#endif

/* Function prototypes section */
void publish(MQTTClient client, char* topic, char* payload);
int on_message(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void run_auto_pilot(float instant_speed);

//variaveis globais
#ifdef ENABLE_PID_DATA_LOGGING_CSV
  unsigned long int pid_points_counter=0;
#endif

/* Function implementation section */ 

/* Function: publish MQTT messages
 * Params: MQTT client, MQTT topic and MQTT payload
 * Return: none 
*/
void publish(MQTTClient client, char* topic, char* payload) {
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = payload;
    pubmsg.payloadlen = strlen(pubmsg.payload);
    pubmsg.qos = 2;
    pubmsg.retained = 0;
    MQTTClient_deliveryToken token;
    MQTTClient_publishMessage(client, topic, &pubmsg, &token);
    MQTTClient_waitForCompletion(client, token, 1000L);
}

/* Function: MQTT received message callback
 * Params: context, pointer to topic name array, topic name length and message received data
 * Return: 1   : success
 *         != 1: fail
*/
int on_message(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    char* payload = message->payload;
    DEBUG_MAIN("Topic: %s Received message: %s\n", topicName, payload);

    /* Parse received message */
    if ( strncmp(topicName, MQTT_AUTOPILOT_SUBSCRIBE_TOPIC, topicLen) == 0)
        setpoint_config(atof(payload));

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

/* Function: run auto-pilot (PID Speed control)
 * Params: instant speed (process variable)
 * Return: none
*/
void run_auto_pilot(float instant_speed)
{
   float nomalized_speed;
   float target_duty_cycle;
   char cmd_line[100];

   nomalized_speed = instant_speed / SPEED_FULL_DUTY_CYCLE;
   target_duty_cycle = calc_PID(nomalized_speed, TIME_STEP_PID_CONTROL);
   rc_set_motor_all(target_duty_cycle);

   //faz log da ação PID (para futura conferencia)
   #ifdef ENABLE_PID_DATA_LOGGING_CSV
      sprintf(cmd_line,"echo '%f,%f,%ld' >> /tmp/pid_data.csv",read_current_setpoint(),target_duty_cycle,pid_points_counter);
      system(cmd_line);
      pid_points_counter++;
   #endif
}

/* Main function section */
int main(int argc, char *argv[])
{
	TStatusExec StatusExec;
    float max_speed=0.0;
    float min_speed=0.0;
    float last_speed=0.0;
    float instant_speed;
    const char speed_units_strings[2][5]={"m/s\0", "km/h\0"};
    time_t ts_print;
    int rc;
    char speed_string[10];
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    /* MQTT init (connection & subscribe) */
    MQTTClient_create(&client, MQTT_ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_setCallbacks(client, NULL, NULL, on_message, NULL);

    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) 
    {
        DEBUG_MAIN("MQTT connection fail. Error code: : %d\n", rc);
        exit(-1);
    }

    MQTTClient_subscribe(client, MQTT_AUTOPILOT_SUBSCRIBE_TOPIC, 0);
 
    /* Measurement variables init and main loop */

	StatusExec = init_speed_by_acc_data();

    if ( StatusExec != eStatusExecOK)
	{
	   DEBUG_MAIN("\r\n[ERRO] Erro ao inicializar calculo de instant_speedocidade pelo acelerometro.\n\rCodigo de erro: %d\n",StatusExec);
	   return 0;
	}

    max_speed = get_instant_speed(SPEED_UNIT);
    min_speed = max_speed;

    /* Motor controllers init */
    rc_enable_motors();
    rc_set_motor_all(0.0);

    DEBUG_MAIN("\r\n[START SAMPLING ACC DATA]\n");

    ts_print = time(NULL);

    /* config setpoint start value */
    setpoint_config(0);

	while (rc_get_state() != EXITING)
	{
        instant_speed = get_instant_speed(SPEED_UNIT);
        last_speed = instant_speed;

        if (instant_speed > max_speed)
            max_speed = instant_speed;

        if (instant_speed < min_speed)
            min_speed = instant_speed;

        
        if ( (time(NULL) - ts_print) >= TIME_STEP_PID_CONTROL)
        {
            DEBUG_MAIN("\r[SPEED INFORMATION] Instant speed: %f %s\n", last_speed, speed_units_strings[SPEED_UNIT]);
            sprintf(speed_string,"%.2f %s", last_speed, speed_units_strings[SPEED_UNIT]);
            publish(client, MQTT_PUBLISH_TOPIC, speed_string);
            ts_print = time(NULL);

            /* run auto-pilot */ 
            run_auto_pilot(instant_speed);
        }

        rc_usleep(TIME_STEP_PID_CONTROL*1000000);
	}

    /* Disable motor controllers */
    rc_disable_motors();

    /* Disable speed calculation by accelerometer data */
	end_speed_calculation_by_acc();
    DEBUG_MAIN("\r\n[SAMPLING TERMINATE] Max: %f %s    Min: %f %s   Last: %f %s\n",max_speed,speed_units_strings[SPEED_UNIT],min_speed,speed_units_strings[SPEED_UNIT],last_speed,speed_units_strings[SPEED_UNIT]);

    /* End MQTT connection */
    MQTTClient_disconnect(client, 1000);
    MQTTClient_destroy(&client);
	return 0;
}