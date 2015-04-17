#ifndef PID_H_
#define PID_H_

#include "lpc_types.h"

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

#define PID_T int

typedef struct {
	PID_T target;

	PID_T p;
	PID_T i;
	PID_T d;

	PID_T max;
	PID_T min;

	PID_T lastError;
	PID_T setpoint;
} MyPid;

//#define MIN(a,b) (((a)<(b))?(a):(b))
//#define MAX(a,b) (((a)>(b))?(a):(b))

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

void pid_init(MyPid*,PID_T,PID_T,PID_T,PID_T,PID_T);
int pid_calc(MyPid*,PID_T);
void pid_set_target(MyPid*,PID_T);


#endif /* PID_H_ */
