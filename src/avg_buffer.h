#ifndef AVG_BUFFER_H_
#define AVG_BUFFER_H_

#include "board.h"

// Initial sampling rate = 1 200 000 Hz
// Desired sampling rate = 2000 Hz
//
#define AVG_SAMPLES 4

typedef struct {
	volatile uint32_t * buffer;
	int head;
	int length;
} MyBuffer;

void init_buffer(volatile MyBuffer*);
//void put_val(uint32_t, int32_t);
int32_t get_avg(volatile MyBuffer*, int32_t);

#endif /* AVG_BUFFER_H_ */
