
#include "board.h"
#include "fir.h"
#include "avg_buffer.h"
#include "power_avg.h"
#include "pid.h"
#include "init.h"
#include "lpc_types.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

//#define PRINT_TEST_BUFFERS

#define BUFFER_A 0x1
#define BUFFER_B 0x2

#define THRESHOLD_PERCENT 90 //((TARGET_POWER/40) + (TARGET_POWER))
#define THRESHOLD (((1 << 20)  / 100) * (THRESHOLD_PERCENT))
#define GAIN_P 1
#define INIT_GAIN 10000

// Desired duty cycle in percent
#define TARGET_DUTY 10
// Number of samples to use to compute threshold level and gain
#define GAIN_AVG 2048
#define TARGET_DUTY_ACCUM (((TARGET_DUTY)*(GAIN_AVG))/100)

#define COLOUR_TIME 20000

#define TEST_BUFFER_SIZE 512

#define MY_ADDR 0
#define NUM_OUTPUTS 3

#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

#define UART_TEST_DEFAULT_BAUDRATE 115200

#define HEADER 0xAA

// Ring buffer size
#define UART_RB_SIZE 64
// Transmit and receive ring buffers
static RINGBUFF_T rxring;
static uint8_t rxbuff[UART_RB_SIZE];

static int32_t adcBuffer[TEST_BUFFER_SIZE];
static int32_t avgBuffer[TEST_BUFFER_SIZE];

static volatile int ticks;
static volatile bool transferComplete;
static volatile int bufferComplete = BUFFER_B;

static MyFilter firFilter;
static MyPwrBuffer powerBuffer;

static MyPid powerPid;

const char inst1[] = "Open OSC Test\r\n";

const int my_outputs[] = {LED_R, LED_G, LED_B};
int my_colours[NUM_OUTPUTS];
int my_beatgates[NUM_OUTPUTS];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

typedef enum {
	OFF = 0,
	RED,
	GREEN,
	BLUE,
	WHITE
} LEDColour;

typedef enum {
	WAIT,
	GET_HDR2,
	GET_ADDR,
	GET_OUT_ADDR,
	GET_CMD,
	GET_HDATA,
	GET_LDATA,
	GET_NEWLINE
} uart_state_t;

typedef enum {
	CMD_OFF    = 0x01,
	CMD_ON     = 0x02,
	CMD_SET    = 0x03,
	CMD_BG_OFF = 0x11,
	CMD_BG_ON  = 0x12,
	CMD_BG_SET = 0x13
} cmd_t;

typedef struct {
	uint8_t  addr;
	uint8_t  out_addr;
	cmd_t    cmd;
	uint16_t data;
	int      valid;
} rx_packet_t;


/*****************************************************************************
 * Function prototypes
 ****************************************************************************/

void loop(void);
void print_device(void);
void print_buffers(void);
void store_debug(int32_t, int32_t);
static void process_input(rx_packet_t *);
static void interpret_packet(rx_packet_t *, int[], int*);
void set_output(int, int);
static inline void set_state(int[], int, int);

/*****************************************************************************
 * Private functions
 ****************************************************************************/
// ADC Rate:
//	14.08 kHz
// After averages (4):
//	3520
//4687


void loop() {
	int32_t firResult_S16;
	int32_t avgResult_S14;
	int32_t pwrResult;
	int32_t dutyCycleError;
	int32_t correctedGain;
	int32_t gainedResult;

	static int32_t gain = INIT_GAIN;
	static int32_t accumDutyCycle;
	static unsigned int sampleCount = 0;
	static unsigned int colourCount = 0;
	static unsigned int myColour = WHITE;
	static unsigned int colourTime = 0;

	static rx_packet_t rx_packet;

	static int32_t maxSig = 0;
	static int32_t minSig = 0x0FFFFFFF;

#ifdef PRINT_TEST_BUFFERS
	// Run for a little while just in case the first few samples are 0 or corrupted
	if (sampleCount >= (10*TEST_BUFFER_SIZE)) {
		// Stop the ADC after collecting data
		Chip_ADC_StopBurstSequencer(LPC_ADC, ADC_SEQA_IDX);
		print_buffers();
		while(1);
	}
#endif

	// Start processing after DMA interrupt
	__WFI();

	process_input(&rx_packet);

	if (rx_packet.valid == TRUE) {
		rx_packet.valid = FALSE;
		interpret_packet(&rx_packet, my_colours, my_beatgates);
	}

	// Is a DMA transfer complete?
	if (transferComplete) {
		transferComplete = false;

		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, 1);

		avgResult_S14 = 0;
		// Get raw sample data from input channel
		// Use the buffer the DMA is not writing to
		for (int i = 0; i < (DMA_BUFFER_SIZE/AVG_SAMPLES); ++i) {
			if (bufferComplete == BUFFER_A) {
				avgResult_S14 = get_avg(&bufferA, ADC_OFFSET);
			} else {
				avgResult_S14 = get_avg(&bufferB, ADC_OFFSET);
			}

			// Run the data through the FIR filter
			// It stores a history, so we can immediately get a result value
			// delayed by the length of the filter
			FirFilter_put(&firFilter, avgResult_S14);
			firResult_S16 = FirFilter_get(&firFilter);

			// Reduce the precision slightly so the size doesn't get too out of control
			// Compute the sliding-window average of the incoming signal to determine
			// The energy content within the window period
			put_pwr_val(&powerBuffer, (firResult_S16*firResult_S16) >> 8);
			pwrResult = get_pwr_avg(&powerBuffer);

			// Calculate the gain adjusted value
			gainedResult = (int32_t)(((int64_t)pwrResult * (int64_t)gain) >> 8);
			// In case of overflow, just set the signal to 0
			if (gainedResult < 0) gainedResult = 0;

			maxSig = MAX(gainedResult, maxSig);
			minSig = MIN(gainedResult, minSig);

			int beat = FALSE;
			if (gainedResult > THRESHOLD) {
				beat = TRUE;
				accumDutyCycle++;
			}

			for (int j = 0; j < 3; ++j) {
				set_output(j, (beat || !my_beatgates[j]) ? my_colours[j] : 0);
			}

			if (++sampleCount > GAIN_AVG) {

				dutyCycleError = TARGET_DUTY_ACCUM - accumDutyCycle;
				correctedGain = gain + dutyCycleError * GAIN_P;

#ifdef DEBUG_DSP
				DEBUGOUT("avg result: %8i | ", avgResult_S14);
				DEBUGOUT("fir result: %8i | ", firResult_S16);
				//DEBUGOUT("fir sq result: %10i\t", (firResult_S16 * firResult_S16) >> 8);
				//DEBUGOUT("pwr result: %8i | ", pwrResult);
				DEBUGOUT("gained: %8i | ", gainedResult);
				DEBUGOUT("accum duty: %4i | ", accumDutyCycle*100/GAIN_AVG);
				//DEBUGOUT("curr gain: %8i | ", gain);
				DEBUGOUT("corrected gain: %8i | ", correctedGain);
				DEBUGOUT("max: %8i | ", maxSig);
				DEBUGOUT("min: %8i | ", minSig);
				DEBUGSTR("\r\n");
#endif

				if (correctedGain > 0) {
					gain = correctedGain;
				}

				sampleCount = 0;
				accumDutyCycle = 0;
				minSig = 0x0FFFFFFF;
				maxSig = 0;
			}

#ifdef PRINT_TEST_BUFFERS
			store_debug(firResult_S16, pwrResult);
#endif

		}
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, 1);
	}
}

static void process_input(rx_packet_t * rx_packet) {
	//const char debug_msg[] = "\nRX Char: ";
	/*const char wait_msg[] = " WAIT_STATE ";
	const char get_addr_msg[] = " ADDR_STATE ";
	const char get_out_addr_msg[] = " OUT_ADDR_STATE ";
	const char get_cmd_msg[] = " GET_CMD_STATE ";
	const char get_hdata_msg[] = " GET_HDATA_STATE ";
	const char get_ldata_msg[] = " GET_LDATA_STATE ";*/


	static uart_state_t state;
	uart_state_t next_state;

	uint8_t input;
	// Grab the next character from the debug UART
	int bytes = Chip_UART_ReadRB(LPC_USART, &rxring, &input, 1);

	//Chip_UART_SendBlocking(LPC_USART, &debug_msg, sizeof(debug_msg) - 1);
	//Chip_UART_SendBlocking(LPC_USART, &input, 1);

	// If there is nothing new to process, just return
	if (bytes != 0) {
		next_state = WAIT;
		switch(state) {
			case WAIT:
				//Chip_UART_SendBlocking(LPC_USART, &wait_msg, sizeof(wait_msg) - 1);
				// See if the input is a header
				if (HEADER == input) {
					rx_packet->valid = FALSE;
					next_state = GET_HDR2;
				}
				break;
			case GET_HDR2:
				if (HEADER == input) {
					next_state = GET_ADDR;
				}
			case GET_ADDR:
				//Chip_UART_SendBlocking(LPC_USART, &get_addr_msg, sizeof(get_addr_msg) - 1);
				if (MY_ADDR == input) {
					rx_packet->addr = input;
					next_state = GET_OUT_ADDR;
				}
				break;
			case GET_OUT_ADDR:
				//Chip_UART_SendBlocking(LPC_USART, &get_out_addr_msg, sizeof(get_out_addr_msg) - 1);
				if ((input <= NUM_OUTPUTS) && (input > 0)) {
					rx_packet->out_addr = input;
					next_state = GET_CMD;
				}
				// Not in the valid output address range for this device
				break;
			case GET_CMD:
				//Chip_UART_SendBlocking(LPC_USART, &get_cmd_msg, sizeof(get_cmd_msg) - 1);
				if (0 != input) {
					rx_packet->cmd = (cmd_t)input;
					next_state = GET_HDATA;
				}
				break;
			case GET_HDATA:
				//Chip_UART_SendBlocking(LPC_USART, &get_hdata_msg, sizeof(get_hdata_msg) - 1);
				rx_packet->data = input << 8;
				next_state = GET_LDATA;
				break;
			case GET_LDATA:
				//Chip_UART_SendBlocking(LPC_USART, &get_ldata_msg, sizeof(get_ldata_msg) - 1);
				rx_packet->data = rx_packet->data | input;
				next_state = GET_NEWLINE;
				break;
			case GET_NEWLINE:
				if ('\n' == input) {
					rx_packet->valid = TRUE;
				}
				break;
			default:
				break;
		}
		state = next_state;
	}
	return;
}

static void interpret_packet(rx_packet_t * rx_packet, int colours[], int bgts[]) {
	switch(rx_packet->cmd) {
		case CMD_OFF:
			set_state(colours, rx_packet->out_addr, 0);
			break;
		case CMD_ON:
			set_state(colours, rx_packet->out_addr, 255);
			break;
		case CMD_SET:
			set_state(colours, rx_packet->out_addr, rx_packet->data & 0xFF);
			break;
		case CMD_BG_OFF:
			set_state(bgts, rx_packet->out_addr, FALSE);
			break;
		case CMD_BG_ON:
			set_state(bgts, rx_packet->out_addr, TRUE);
			break;
		default:
			break;
	}
}

static inline void set_state(int state_buffer[], int addr, int val) {
	state_buffer[addr - 1] = val;
}

void set_output(int idx, int setpoint) {
	uint32_t max_ticks = Chip_SCTPWM_GetTicksPerCycle(SCT_PWM);
	uint32_t set_ticks = max_ticks * (255 - setpoint) / 255;

	Chip_SCTPWM_SetDutyCycle(SCT_PWM, my_outputs[idx], set_ticks);
}

void print_device() {
	DEBUGSTR("\r\nAudio DSP test\r\n");
	DEBUGOUT("System Clock: %uMHz\r\n", SystemCoreClock / 1000000);
	DEBUGOUT("Device ID: 0x%x\r\n", Chip_SYSCTL_GetDeviceID());
	DEBUGOUT("ADC Sampling Rate: %u\r\n", 1200000/ADC_SAMPLING_RATE_DIV);
	DEBUGOUT("Effective Sampling Rate: %u\r\n", 1200000/ADC_SAMPLING_RATE_DIV/AVG_SAMPLES);
}

void store_debug(int32_t adc, int32_t avg) {
	static unsigned int adcHead;
	static unsigned int avgHead;

	if (adcHead < (TEST_BUFFER_SIZE-1)) {
		++adcHead;
	} else {
		adcHead = 0;
 	}
	adcBuffer[adcHead] = adc;

	if (avgHead < (TEST_BUFFER_SIZE-1)) {
		++avgHead;
	} else {
		avgHead = 0;
	 }
	avgBuffer[avgHead] = avg;
}

void print_buffers() {

	DEBUGSTR("*****Printing buffers*****\r\n\r\n");
	DEBUGSTR("adc = [");
	for(int i = 0; i < TEST_BUFFER_SIZE; ++i) {
		DEBUGOUT("%d,",adcBuffer[i]);
	}
	DEBUGSTR("]; avg = [");
	for(int i = 0; i < TEST_BUFFER_SIZE; ++i) {
		DEBUGOUT("%d,",avgBuffer[i]);
	}
	DEBUGSTR("];\r\n\r\n*****      Done       *****\r\n\r\n");
}


/*****************************************************************************
 * Interrupt handlers
 ****************************************************************************/

// Handle interrupt from SysTick timer
void SysTick_Handler(void)
{
	static uint32_t count;

	if (count++ >= TICKRATE_HZ) {
		count = 0;
		Board_LED_Toggle(0);
		//Chip_UART_SendBlocking(LPC_USART, inst1, sizeof(inst1) - 1);
	}
}

void LPC_UARTHNDLR(void)
{
	// Use NXP provided ring buffer handler
	Chip_UART_RXIntHandlerRB(LPC_USART, &rxring);
}

#ifndef USE_DMA
// Handle interrupt from ADC sequencer A
void ADC_SEQA_IRQHandler(void)
{
	uint32_t pending;
	// Get pending interrupts
	pending = Chip_ADC_GetFlags(LPC_ADC);

	// Toggle GPIO to see ADC rate
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, 0);

	// Sequence A completion interrupt
	if (pending & ADC_FLAGS_SEQA_INT_MASK) {
		transferComplete = true;
	}

	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, 0);

	// Clear any pending ADC interrupts
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}
#endif

void DMA_IRQHandler(void)
{
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, 0);
	// Error interrupt on channel 0?
	if ((Chip_DMA_GetIntStatus(LPC_DMA) & DMA_INTSTAT_ACTIVEERRINT) != 0) {
		// This shouldn't happen for this simple DMA example, so set the LED
		//   to indicate an error occurred. This is the correct method to clear
		//   an abort
		Chip_DMA_DisableChannel(LPC_DMA, DMA_CH0);
		while ((Chip_DMA_GetBusyChannels(LPC_DMA) & (1 << DMA_CH0)) != 0) {}
		Chip_DMA_AbortChannel(LPC_DMA, DMA_CH0);
		Chip_DMA_ClearErrorIntChannel(LPC_DMA, DMA_CH0);
		Chip_DMA_EnableChannel(LPC_DMA, DMA_CH0);
		//while(1); // Block on error
	}

	// Clear DMA interrupt for the channel
	Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMA_CH0);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, 0);
	if (bufferComplete == BUFFER_A) {
		bufferComplete = BUFFER_B;
	} else {
		bufferComplete = BUFFER_A;
	}
	transferComplete = true;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

int main(void)
{
	// Zero out the two DMA target buffers before starting
	init_buffer(&bufferA);
	init_buffer(&bufferB);

	RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);

	init_pwr_buffer(&powerBuffer);
	FirFilter_init(&firFilter);

	//pid_init(&powerPid, 10000, 0, 0, 1<<16, -(1<<16));
	//pid_set_target(&powerPid, TARGET_POWER);

	init();

	print_device();

	// Enable the clock to the Switch Matrix
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 4);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 0);

	// Disable the clock to the Switch Matrix to save power
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_UART_Init(LPC_USART);
	Chip_UART_ConfigData(LPC_USART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_Clock_SetUSARTNBaseClockRate((115200 * 16), true);
	Chip_UART_SetBaud(LPC_USART, UART_TEST_DEFAULT_BAUDRATE);
	Chip_UART_Enable(LPC_USART);
	Chip_UART_TXEnable(LPC_USART);

	Chip_UART_IntEnable(LPC_USART, UART_INTEN_RXRDY);

	NVIC_EnableIRQ(LPC_IRQNUM);

	Chip_UART_SendBlocking(LPC_USART, inst1, sizeof(inst1) - 1);

	// Loop forever
	while (1) {
		loop();
	}
}


