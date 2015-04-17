#include "fir.h"

static int32_t filter_taps[FILTER_TAP_NUM] = {
		  -1485,
		  -1542,
		  -2122,
		  -2643,
		  -3004,
		  -3112,
		  -2892,
		  -2304,
		  -1353,
		  -99,
		  1355,
		  2866,
		  4272,
		  5414,
		  6159,
		  6418,
		  6159,
		  5414,
		  4272,
		  2866,
		  1355,
		  -99,
		  -1353,
		  -2304,
		  -2892,
		  -3112,
		  -3004,
		  -2643,
		  -2122,
		  -1542,
		  -1485
};

void FirFilter_init(MyFilter* f) {
	for(int i = 0; i < FILTER_TAP_NUM; ++i) {
		f->history[i] = 0;
	}

	f->last_index = 0;
}

void FirFilter_put(MyFilter* f, int32_t input) {
	f->history[f->last_index++] = input;

	if(f->last_index == FILTER_TAP_NUM) {
		f->last_index = 0;
	}
}

int32_t FirFilter_get(MyFilter* f) {
	int64_t acc = 0;
	uint32_t index = f->last_index;

	for(int i = 0; i < FILTER_TAP_NUM; ++i) {
		index = index != 0 ? index-1 : FILTER_TAP_NUM-1;
		acc += (int64_t)f->history[index] * filter_taps[i];
	}

	return acc >> 10;//16;
}
