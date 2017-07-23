/*
===============================================================================
 Name        : LPC824_MAX30102.c
 Author      : Joe Desbonnet
 Version     :
 Copyright   : $(copyright)
 Description :
 Interface to MAX30102 SPO2 sensor.
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>


//
// Hardware configuration
//
//#define UART_BAUD_RATE 115200
#define UART_BAUD_RATE 230400

// Define function to pin mapping. Pin numbers here refer to PIO0_n
// and is not the same as a package pin number.
// Use PIO0_0 and PIO0_4 for UART RXD, TXD (same as ISP)
#define PIN_UART_RXD 0
#define PIN_UART_TXD 4
// General purpose debug pin
#define PIN_DEBUG 14

#define PIN_INT 13
#define PIN_INT0 13
#define PIN_INT1 13


#define PIN_I2C_SCL 10
#define PIN_I2C_SDA 13
#define PIN_I2C0_SCL 10
#define PIN_I2C0_SDA 13
#define PIN_I2C1_SCL 8
#define PIN_I2C1_SDA 9

#define PIN_LED 12

/* 100kbps I2C bit-rate */
#define I2C_BITRATE             (100000)

// Data available flags for both I2C devices (set in ISR).
static volatile uint8_t data_available_flag[2] = {0,0};
static volatile uint32_t data_timestamp[2] = {0,0};

#define FILTER_TAP_NUM 73

extern int filter_taps[FILTER_TAP_NUM];

/**
 * @brief Send one byte to UART. Block if UART busy.
 * @param n Byte to send to UART.
 * @return None.
 */
static void print_byte (uint8_t n) {
	//Chip_UART_SendBlocking(LPC_USART0, &n, 1);

	// Wait until data can be written to FIFO (TXRDY==1)
	while ( (Chip_UART_GetStatus(LPC_USART0) & UART_STAT_TXRDY) == 0) {}

	Chip_UART_SendByte(LPC_USART0, n);
}

/**
 * @brief Print a signed integer in decimal radix.
 * @param n Number to print.
 * @return None.
 */
static void print_decimal (int n) {
	char buf[10];
	int i = 0;

	// Special case of n==0
	if (n == 0) {
		print_byte('0');
		return;
	}

	// Handle negative numbers
	if (n < 0) {
		print_byte('-');
		n = -n;
	}

	// Use modulo 10 to get least significant digit.
	// Then /10 to shift digits right and get next least significant digit.
	while (n > 0) {
		buf[i++] = '0' + n%10;
		n /= 10;
	}

	// Output digits in reverse order
	do {
		print_byte (buf[--i]);
	} while (i>0);

}

/**
 * @brief Print uint32 integer in decimal radix.
 * @param n Number to print.
 * @return None.
 */
static void print_decimal_uint32 (uint32_t n) {
	char buf[10];
	int i = 0;

	// Special case of n==0
	if (n == 0) {
		print_byte('0');
		return;
	}

	// Use modulo 10 to get least significant digit.
	// Then /10 to shift digits right and get next least significant digit.
	while (n > 0) {
		buf[i++] = '0' + n%10;
		n /= 10;
	}

	// Output digits in reverse order
	do {
		print_byte (buf[--i]);
	} while (i>0);

}

void setup_pin_for_interrupt (int interrupt_pin, int interrupt_channel) {

	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, interrupt_pin);
	Chip_IOCON_PinSetMode(LPC_IOCON,interrupt_pin,PIN_MODE_PULLUP);

	/* Configure interrupt channel for the GPIO pin in SysCon block */
	Chip_SYSCTL_SetPinInterrupt(interrupt_channel, interrupt_pin);

	/* Configure GPIO pin as input pin */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, interrupt_pin);

	/* Configure channel 7 interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(interrupt_channel));
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(interrupt_channel));
}


void setup_sct_for_timer (void) {

	Chip_SCT_Init(LPC_SCT);

	/* Stop the SCT before configuration */
	Chip_SCTPWM_Stop(LPC_SCT);

	// Match/capture mode register. (ref UM10800 section 16.6.11, Table 232, page 273)
	// Determines if match/capture operate as match or capture. Want all match.
	LPC_SCT->REGMODE_U = 0;

	// Set SCT Counter to count 32-bits and reset to 0 after reaching MATCH0
	Chip_SCT_Config(LPC_SCT, SCT_CONFIG_32BIT_COUNTER );

	Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);
}

setup_max30102 (LPC_I2C_T *i2c) {

	// Reset
	hw_i2c_register_write(i2c, 0x9, 1<<6);
	hw_i2c_register_write(i2c, 0x9, 1<<6);

	// Interrupt Enable 1 Register. Set PPG_RDY_EN (data available in FIFO)
	hw_i2c_register_write(i2c, 0x2, 1<<6);

	// FIFO configuration register
	// SMP_AVE: 16 samples averaged per FIFO sample
	// FIFO_ROLLOVER_EN=1
	//hw_i2c_register_write(0x8,  1<<4);
	hw_i2c_register_write(i2c, 0x8, (0<<5) | 1<<4);

	// Mode Configuration Register
	// SPO2 mode
	hw_i2c_register_write(i2c, 0x9, 3);

	// SPO2 Configuration Register
	hw_i2c_register_write(i2c, 0xa,
			(3<<5)  // SPO2_ADC_RGE 2 = full scale 8192 nA (LSB size 31.25pA); 3 = 16384nA
			| (1<<2) // sample rate: 0 = 50sps; 1 = 100sps; 2 = 200sps
			| (3<<0) // LED_PW 3 = 411µs, ADC resolution 18 bits
	);

	// LED1 (red) power (0 = 0mA; 255 = 50mA)
	hw_i2c_register_write(i2c, 0xc, 0xb0);

	// LED (IR) power
	hw_i2c_register_write(i2c, 0xd, 0xa0);
}

/**
 * @brief	Handle interrupt from PININT6
 * @return	Nothing
 */
void PININT6_IRQHandler(void)
{

	// Clock from SCT counter
	data_timestamp[1] = LPC_SCT->COUNT_U;
	data_available_flag[1]=1;
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH6);
}

/**
 * @brief	Handle interrupt from PININT7
 * @return	Nothing
 */
void PININT7_IRQHandler(void)
{
	// Clock from SCT counter
	data_timestamp[0] = LPC_SCT->COUNT_U;
	data_available_flag[0]=1;
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH7);
}

void retrieve_sample (LPC_I2C_T *i2c_bus, int32_t *fifo_write_ptr, int32_t *last_fifo_write_ptr, int32_t *red, int32_t *nir) {

	// Clear interrupt register by reading it
	uint8_t intreg = hw_i2c_register_read(i2c_bus,0x0);

	// Get FIFO write pointer
	*fifo_write_ptr = hw_i2c_register_read(i2c_bus, 0x4);

	uint8_t buf[6];

	if (fifo_write_ptr != last_fifo_write_ptr) {
		// Clock from SCT counter
		//ts = LPC_SCT->COUNT_U;
		hw_i2c_fifo_read(i2c_bus, buf,6);
		*red = (buf[0]<<16) | (buf[1]<<8) | buf[2];
		*nir = (buf[3]<<16) | (buf[4]<<8) | buf[5];
	}
}

int main(void) {

#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif

    int i;

	// Enable I2C0 fixed location pins and second I2C1 on movable pins.
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	// I2C0 (fast fixed I2C) for first sensor
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);

	// I2C1 for second sensor
	Chip_SWM_MovablePinAssign(SWM_I2C1_SCL_IO, PIN_I2C1_SCL);
	Chip_SWM_MovablePinAssign(SWM_I2C1_SDA_IO, PIN_I2C1_SDA);

	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);


    //
    // Initialize GPIO
    //
	Chip_GPIO_Init(LPC_GPIO_PORT);

	// Configure interrupt pins
	setup_pin_for_interrupt(PIN_INT, 7);
	setup_pin_for_interrupt(PIN_INT, 6);

	/* Enable interrupt in the NVIC */
	NVIC_EnableIRQ(PININT6_IRQn);
	NVIC_EnableIRQ(PININT7_IRQn);



	//
	// Initialize UART
	//

	// Assign pins: use same assignment as serial bootloader
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, PIN_UART_TXD);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, PIN_UART_RXD);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_UART_Init(LPC_USART0);
	Chip_UART_ConfigData(LPC_USART0,
			UART_CFG_DATALEN_8
			| UART_CFG_PARITY_NONE
			| UART_CFG_STOPLEN_1);

	Chip_Clock_SetUSARTNBaseClockRate((UART_BAUD_RATE * 16), true);
	Chip_UART_SetBaud(LPC_USART0, UART_BAUD_RATE);
	Chip_UART_TXEnable(LPC_USART0);
	Chip_UART_Enable(LPC_USART0);

	//
	// Initialize GPIO
	//
	//Chip_GPIO_Init(LPC_GPIO_PORT);
	setup_sct_for_timer();
	uint32_t clock_hz = Chip_Clock_GetSystemClockRate();


	hw_i2c_setup(LPC_I2C0);
	hw_i2c_setup(LPC_I2C1);

	setup_max30102(LPC_I2C0);
	setup_max30102(LPC_I2C1);

	uint32_t v_red,v_nir;

	int32_t lpf_red[2] = {0,0};
	int32_t lpf_nir[2] = {0,0};

	uint32_t sample_counter=0;
	uint32_t led_off_time;

	uint8_t buf[6];

	uint8_t fifo_read_ptr[2];
	uint8_t fifo_write_ptr[2];

	uint32_t device_index;
	LPC_I2C_T *device;

	LPC_I2C_T *device_i2c_bus[2];
	device_i2c_bus[0] = LPC_I2C0;
	device_i2c_bus[1] = LPC_I2C1;

	fifo_read_ptr[0] =  hw_i2c_register_read(LPC_I2C0, 0x6);
	fifo_read_ptr[1] =  hw_i2c_register_read(LPC_I2C1, 0x6);

	uint8_t last_fifo_write_ptr[2]={0,0};

	uint8_t intreg;


	// Note about datarates: LPC824 I2C0 can operate up to 1Mpbs, Other I2C
	// interfaces are limited to 400kbps. Each sample requires 2 register
	// reads and 6 bytes of sample data. 64bits x 100sps = 6.4kbps
    while(1) {

    	// Wait for data-available interrupt
    	while (data_available_flag[0]==0 && data_available_flag[1]==0) {
    		__WFI();
    	}

    	if (data_available_flag[0] != 0) {
    		device_index=0;
    	} else {
    		device_index=1;
    	}

    	data_available_flag[device_index] = 0;
    	device = device_i2c_bus[device_index];

    	// Read interrupt register
    	intreg = hw_i2c_register_read(device,0x0);

    	// Get FIFO write pointer
    	fifo_write_ptr[device_index] = hw_i2c_register_read(device, 0x4);

    	if (fifo_write_ptr[device_index] != last_fifo_write_ptr[device_index]) {

    		hw_i2c_fifo_read(device, buf,6);

    		v_red = (buf[0]<<16) | (buf[1]<<8) | buf[2];
    		v_nir = (buf[3]<<16) | (buf[4]<<8) | buf[5];

    		lpf_red[device_index] = (v_red + 15*lpf_red[device_index])/16;
    		lpf_nir[device_index] = (v_nir + 15*lpf_nir[device_index])/16;

            // output format:
            // timestamp (us)
    		// device
            // red
            // nir


            // Timestamp in microseconds (timer clocked by 30MHz clock).
    		print_decimal_uint32(data_timestamp[device_index]/30);
    		print_byte(' ');
    		print_decimal(device_index);
    		print_byte(' ');
    		print_decimal(v_red);
    		print_byte(' ');
    		print_decimal(v_nir);


    		print_byte(' ');
    		print_decimal(v_red - lpf_red[device_index]);
    		print_byte(' ');
    		print_decimal(v_nir - lpf_nir[device_index]);


    		print_byte('\r');
    		print_byte('\n');

    		fifo_read_ptr[device_index]++;
    		last_fifo_write_ptr[device_index] = fifo_write_ptr[device_index];

    		sample_counter++;
    	}

    }
    return 0 ;
}
