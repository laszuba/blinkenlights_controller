
#include "avg_buffer.h"

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

void init_buffer(volatile MyBuffer* inputBuffer) {
	inputBuffer->head = 0;

	// zero buffer
	for(int i = 0; i < inputBuffer->length; ++i) {
		inputBuffer->buffer[i] = 0;
	}
}

// Place a sample at the next buffer position
/*void put_val(uint32_t val, int32_t offset) {
	// Wrap the head pointer around to the start
	// if we hit the end of the buffer
	if (head < (BUFFER_LENGTH-1)) {
		++head;
	} else {
		head = 0;
	}
	inputBuffer[head] = val-offset;
}*/

// Gets the current average value from the buffer
int32_t get_avg(volatile MyBuffer* inputBuffer, int32_t offset) {
	int32_t result = 0;//-(4 * offset);
	int length = inputBuffer->length;
	int head = inputBuffer->head;
	int readOffset;

	// Accumulate the samples from the buffer up to AVG_SAMPLES
	for (int i = 0; i < AVG_SAMPLES; ++i) {
		readOffset = head - i;

		// Read the value, converting from raw ADC result to integer
		if (readOffset >= 0) {
			result += (ADC_DR_RESULT(inputBuffer->buffer[readOffset]) - offset);
		} else {
			// Wrap around to the end of the buffer if head too close to the start
			result += (ADC_DR_RESULT(inputBuffer->buffer[length + readOffset]) - offset);
		}
	}

	if (head < (length-AVG_SAMPLES)) {
		inputBuffer->head += AVG_SAMPLES;
	} else {
		inputBuffer->head = 0;
	}

	// Take the actual average and return it
	return result / AVG_SAMPLES;
}
