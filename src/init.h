#ifndef BLINKENLIGHTS_CONTROLLER_SRC_INIT_H_
#define BLINKENLIGHTS_CONTROLLER_SRC_INIT_H_

#include "board.h"
#include "avg_buffer.h"

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

void init(void);

/*****************************************************************************
 * Public Variables
 ****************************************************************************/

volatile MyBuffer bufferA;
volatile MyBuffer bufferB;

/*****************************************************************************
 * Misc Definitions
 ****************************************************************************/

//#define USE_EXT_UART

/*****************************************************************************
 * ADC Configuration
 ****************************************************************************/

#define INPUT_ADC_CH 1
#define ADC_OFFSET 1461
#define ADC_SAMPLING_RATE_DIV 64

/*****************************************************************************
 * PWM Configuration
 ****************************************************************************/

#define SCT_PWM            LPC_SCT
#define SCT_PWM_PIN_OUT    1		/* COUT1 Generate square wave */
#define SCT_PWM_PIN_LED    0		/* COUT0 [index 2] Controls LED */
#define SCT_PWM_OUT        2		/* Index of OUT PWM */
#define SCT_PWM_LED        1		/* Index of LED PWM */
#define SCT_PWM_RATE     100		/* PWM frequency 10 KHz */

#define LED_R_PIN 0
#define LED_G_PIN 1
#define LED_B_PIN 2

#define LED_R 1
#define LED_G 2
#define LED_B 3

// LED Colours:
// 0 (P0_12): red
// 1 (P0_16): green
// 2 (P0_23): blue

/*****************************************************************************
 * System Timer Configuration
 ****************************************************************************/

#define TICKRATE_HZ (10) // 10 ticks per second

/*****************************************************************************
 * DMA Configuration
 ****************************************************************************/

#define USE_DMA

#define ALIGN(x) __attribute__ ((aligned(x)))

// Max 1024 transfers limits buffer size to 1024
// Should be a power of 2
#define DMA_BUFFER_SIZE 128

#define DMA_XFER_CONFIG (DMA_XFERCFG_CFGVALID | \
						DMA_XFERCFG_RELOAD | \
						DMA_XFERCFG_SETINTA | \
						DMA_XFERCFG_WIDTH_32 | \
						DMA_XFERCFG_SRCINC_0 | \
						DMA_XFERCFG_DSTINC_1 | \
						DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE))

#endif /* BLINKENLIGHTS_CONTROLLER_SRC_INIT_H_ */
