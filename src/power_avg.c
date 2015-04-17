
#include "power_avg.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Public types/enumerations/variables
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

void init_pwr_buffer(MyPwrBuffer* inputBuffer) {
	inputBuffer->head = 0;
	inputBuffer->lastAverage = 0;

	// zero buffer
	for(int i = 0; i < WINDOW; ++i) {
		inputBuffer->buffer[i] = 0;
	}
}

// Place a sample at the next buffer position
// The value should be squared to calculate power correctly
void put_pwr_val(MyPwrBuffer* inputBuffer, int32_t val) {
	int32_t oldestVal;
	// Wrap the head pointer around to the start
	// if we hit the end of the buffer
	if (inputBuffer->head < (WINDOW-1)) {
		++inputBuffer->head;
	} else {
		inputBuffer->head = 0;
	}
	// Take the item previously at head, it is currently the oldest item in the buffer
	oldestVal = inputBuffer->buffer[inputBuffer->head];
	inputBuffer->lastAverage += (int64_t)(val - oldestVal);
	inputBuffer->buffer[inputBuffer->head] = val;
}

// Gets the current average value from the buffer
int32_t get_pwr_avg(MyPwrBuffer* inputBuffer) {
	return (inputBuffer->lastAverage) / WINDOW;
}
