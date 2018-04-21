/*
Header do modulo de obtenção de velocidade mediante leituras do acelerometro
Data: Abril/2018
Autor: Pedro Bertoleti
*/

#ifndef SPEEDBYACC_H_
#define SPEEDBYACC_H_

/* Define section */ 
#define M_PER_SEC   0
#define KM_PER_HOUR 1

/* Typedefs section */ 
typedef enum {
	eStatusExecOK = 0,
	eStatusExecFailRootPrivillege = -1,
	eStatusExecFailInitIMU = -2,
}TStatusExec;

/* Global function prototypes */ 
TStatusExec init_speed_by_acc_data(void);
void end_speed_calculation_by_acc(void);
float get_instant_speed(char speed_unit);

#endif
