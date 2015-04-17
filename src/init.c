
#include "init.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

volatile uint32_t dmaBuffA[DMA_BUFFER_SIZE];
volatile uint32_t dmaBuffB[DMA_BUFFER_SIZE];

volatile MyBuffer bufferA = {
		.buffer = dmaBuffA,
		.length = DMA_BUFFER_SIZE,
		.head = 0
};

volatile MyBuffer bufferB = {
		.buffer = dmaBuffB,
		.length = DMA_BUFFER_SIZE,
		.head = 0
};


// DMA descriptors for ADC to SRAM transfer
// See Ref Manual pg 166
ALIGN(16) static DMA_CHDESC_T dmaDescChan;
ALIGN(16) static DMA_CHDESC_T dmaDescA;
ALIGN(16) static DMA_CHDESC_T dmaDescB;

ALIGN(16) static DMA_CHDESC_T dmaDescChan = {
		.xfercfg = DMA_XFER_CONFIG,
		.source = DMA_ADDR(&(LPC_ADC->DR[INPUT_ADC_CH])),
		.dest = DMA_ADDR(&dmaBuffA[DMA_BUFFER_SIZE-1]),
		.next = DMA_ADDR(&dmaDescB)
};

ALIGN(16) static DMA_CHDESC_T dmaDescA = {
		.xfercfg = DMA_XFER_CONFIG,
		.source = DMA_ADDR(&(LPC_ADC->DR[INPUT_ADC_CH])),
		.dest = DMA_ADDR(&dmaBuffA[DMA_BUFFER_SIZE-1]),
		.next = DMA_ADDR(&dmaDescB)
};

ALIGN(16) static DMA_CHDESC_T dmaDescB = {
		.xfercfg = DMA_XFER_CONFIG,
		.source = DMA_ADDR(&(LPC_ADC->DR[INPUT_ADC_CH])),
		.dest = DMA_ADDR(&dmaBuffB[DMA_BUFFER_SIZE-1]),
		.next = DMA_ADDR(&dmaDescA)
};

/*****************************************************************************
 * Private function prototypes
 ****************************************************************************/

static void config_adc_dma(void);
static void config_adc(void);
static void config_pwm(void);

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static void config_adc_dma() {
	// Enable DMA clock and reset if needed
	Chip_DMA_Init(LPC_DMA);

	// Enable DMA controller and use driver provided DMA table for current descriptors
	Chip_DMA_Enable(LPC_DMA);
	Chip_DMA_SetSRAMBase(LPC_DMA, DMA_ADDR(Chip_DMA_Table));

	// Trigger source: ADC_SEQA_IRQ
	// Setup trigger mux for ADC->DMA_CH0
	Chip_DMATRIGMUX_SetInputTrig(LPC_DMATRIGMUX, DMA_CH0, DMATRIG_ADC_SEQA_IRQ);

	// Setup channel 0 for the following configuration:
	//   - High channel priority
	//   - Interrupt A fires on descriptor completion
	//   - One transfer per burst
	//   - Rising edge on trigger from MUX (which is connected to ADC)
	Chip_DMA_EnableChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_EnableIntChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_SetupChannelConfig(LPC_DMA, DMA_CH0,
								(DMA_CFG_HWTRIGEN |
								 DMA_CFG_TRIGTYPE_EDGE |
								 DMA_CFG_TRIGPOL_HIGH |
								 DMA_CFG_TRIGBURST_BURST |
								 DMA_CFG_BURSTPOWER_1 |
								 DMA_CFG_CHPRIORITY(0)));

	// Setup transfer descriptor and validate it
	Chip_DMA_SetupTranChannel(LPC_DMA, DMA_CH0, &dmaDescChan);

	Chip_DMA_SetupChannelTransfer(LPC_DMA, DMA_CH0, dmaDescChan.xfercfg);

	Chip_DMA_SetValidChannel(LPC_DMA, DMA_CH0);

	// Enable DMA interrupt
	NVIC_EnableIRQ(DMA_IRQn);

}

static void config_adc() {
	// Setup ADC for 12-bit mode and normal power
	Chip_ADC_Init(LPC_ADC, 0);

	// Calibrate ADC and wait for result
	Chip_ADC_StartCalibration(LPC_ADC);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC))) {}

	// Setup ADC clock rate to a fraction of SystemCoreClock
	Chip_ADC_SetClockRate(LPC_ADC, /*ADC_MAX_SAMPLE_RATE*/SystemCoreClock/ADC_SAMPLING_RATE_DIV);

	// Setup sequencer A for ADC channel 0, EOS interrupt
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX,
							(ADC_SEQ_CTRL_CHANSEL(INPUT_ADC_CH) | ADC_SEQ_CTRL_MODE_EOS));

	// Clear all pending interrupts
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

	// Enable ADC sequence A completion interrupts, leave overrun off
	// This will trigger the DMA if in use on completion
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE /*| ADC_INTEN_OVRRUN_ENABLE*/));

	// Only enable the interrupt if not using the DMA to transfer data
#ifndef USE_DMA
	// Enable ADC NVIC interrupt
	NVIC_EnableIRQ(ADC_SEQA_IRQn);
#endif

	// Enable sequencer
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);

	// Start the sequencer in burst (running continuously) mode
	Chip_ADC_StartBurstSequencer(LPC_ADC, ADC_SEQA_IDX);
}

static void config_pwm() {
	// Initialize the SCT for PWM use and set frequency
	Chip_SCTPWM_Init(LPC_SCT);
	Chip_SCTPWM_SetRate(LPC_SCT, SCT_PWM_RATE);

	// Connect SCT output 0 to LED pin PIO7, SCT output 1 to PIO17
	//Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 12);
	//Chip_SWM_MovablePinAssign(SWM_SCT_OUT1_O, 16);
	//Chip_SWM_MovablePinAssign(SWM_SCT_OUT2_O, 27);


	Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 20);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT1_O, 21);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT2_O, 22);

	/* Use SCT0_OUT1 pin */
	//Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT, SCT_PWM_PIN_OUT);
	Chip_SCTPWM_SetOutPin(SCT_PWM, LED_R, LED_R_PIN);
	Chip_SCTPWM_SetOutPin(SCT_PWM, LED_G, LED_G_PIN);
	Chip_SCTPWM_SetOutPin(SCT_PWM, LED_B, LED_B_PIN);

	/* Start with 50% duty cycle */
	//Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, LED_R, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, LED_G, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, LED_B, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);
	Chip_SCTPWM_Start(LPC_SCT);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void init() {
	SystemCoreClockUpdate();
	Board_Init();

	// Enable the clock to the Switch Matrix
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	//Board_LED_Set(0, false);
	//Board_LED_Set(1, false);

	// Configure the SWM for P0_6 as the input for the ADC1
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC1);

	// 824 needs the fixed pin ACMP2 pin disabled to use pin as gpio
	Chip_SWM_DisableFixedPin(SWM_FIXED_CLKIN);

	// Set port 0 pins 0,1 to outputs
	Chip_GPIO_SetPortDIROutput(LPC_GPIO_PORT, 0, 0x03);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, 0);

	config_pwm();

	config_adc_dma();

	config_adc();

	// Disable the clock to the Switch Matrix to save power
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	// Setup systick at the desired rate
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
}
