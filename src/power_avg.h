#ifndef BLINKENLIGHTS_CONTROLLER_SRC_POWER_AVG_H_
#define BLINKENLIGHTS_CONTROLLER_SRC_POWER_AVG_H_

#include "board.h"

#define WINDOW 32

typedef struct {
	int32_t buffer[WINDOW];
	int head;
	int64_t lastAverage;
} MyPwrBuffer;

void init_pwr_buffer(MyPwrBuffer*);
void put_pwr_val(MyPwrBuffer*,int32_t);
int32_t get_pwr_avg(MyPwrBuffer*);

#endif /* BLINKENLIGHTS_CONTROLLER_SRC_POWER_AVG_H_ */
