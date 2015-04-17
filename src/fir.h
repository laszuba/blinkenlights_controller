#ifndef FIR_H_
#define FIR_H_

#include "board.h"

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 3520 Hz

fixed point precision: 16 bits

* 0 Hz - 1 Hz
  gain = 0
  desired attenuation = -20 dB
  actual attenuation = n/a

* 100 Hz - 200 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 300 Hz - 1760 Hz
  gain = 0
  desired attenuation = -30 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM 31

typedef struct {
	int32_t history[FILTER_TAP_NUM];
	uint32_t last_index;
} MyFilter;

void FirFilter_init(MyFilter* f);
void FirFilter_put(MyFilter* f, int32_t input);
int32_t FirFilter_get(MyFilter* f);

#endif /* FIR_H_ */
