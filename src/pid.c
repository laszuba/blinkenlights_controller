
#include "pid.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private function prototypes
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void pid_init(MyPid* my_pid, PID_T p, PID_T i, PID_T d, PID_T max, PID_T min) {
	my_pid->p = p;
	my_pid->i = i;
	my_pid->d = d;

	my_pid->max = max;
	my_pid->min = min;

	my_pid->target = 0;
}

PID_T pid_calc(MyPid* my_pid, PID_T current) {
	my_pid->lastError = my_pid->target - current;

	my_pid->setpoint = ((my_pid->lastError >> 4) * my_pid->p) >> 12;

	return MAX(MIN(my_pid->setpoint, my_pid->max), my_pid->min);
}

void pid_set_target(MyPid* my_pid, PID_T target) {
	my_pid->target = target;
}
