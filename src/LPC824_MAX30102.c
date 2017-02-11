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


#define PIN_I2C_SCL 10
#define PIN_I2C_SDA 13

/* 100kbps I2C bit-rate */
#define I2C_BITRATE             (100000)

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

    //
    // Initialize GPIO
    //
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, PIN_INT);
	Chip_IOCON_PinSetMode(LPC_IOCON,PIN_INT,PIN_MODE_PULLUP);

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

	hw_i2c_setup();

	// Reset
	hw_i2c_register_write(0x9, 1<<6);
	hw_i2c_register_write(0x9, 1<<6);

	// Interrupt Enable 1 Register. Set PPG_RDY_EN (data available in FIFO)
	hw_i2c_register_write(0x2, 1<<6);

	// FIFO configuration register
	// SMP_AVE: 16 samples averaged per FIFO sample
	// FIFO_ROLLOVER_EN=1
	//hw_i2c_register_write(0x8,  1<<4);
	hw_i2c_register_write(0x8, (0<<5) | 1<<4);

	// Mode Configuration Register
	// SPO2 mode
	hw_i2c_register_write(0x9, 3);

	// SPO2 Configuration Register
	hw_i2c_register_write(0xa,
			(3<<5)  // SPO2_ADC_RGE 2 = full scale 8192 nA (LSB size 31.25pA); 3 = 16384nA
			| (0<<2) // sample rate: 0 = 50sps; 1 = 100sps; 2 = 200sps
			| (3<<0) // LED_PW 3 = 411µs, ADC resolution 18 bits
	);

	// LED1 (red) power (0 = 0mA; 255 = 50mA)
	hw_i2c_register_write(0xc, 0xa0);

	// LED (IR) power
	hw_i2c_register_write(0xd, 0xa0);

	uint8_t v0,v1,v2;
	uint8_t fifo_read_ptr =  hw_i2c_register_read(0x6);
	uint8_t fifo_write_ptr;
	uint8_t intreg;
	uint8_t last_fifo_write_ptr=0;
	uint8_t nsamples;
	uint32_t v_red,v_ir;
	uint32_t ts, prev_ts=0;

	int32_t lpf_red=0, lpf_ir=0;

	int npoll;

	uint8_t buf[6];

    while(1) {
    	// Get FIFO write pointer
    	fifo_write_ptr = hw_i2c_register_read(0x4);
    	//i2c_delay();

    	// Read interrupt register
    	intreg = hw_i2c_register_read(0x0);

    	//nsamples = fifo_write_ptr - fifo_read_ptr;

    	//for (i = 0; i < nsamples; i++) {
    	if (fifo_write_ptr != last_fifo_write_ptr) {
    		ts = LPC_SCT->COUNT_U;

    		hw_i2c_fifo_read(buf,6);
    		v_red = (buf[0]<<16) | (buf[1]<<8) | buf[2];
    		v_ir = (buf[3]<<16) | (buf[4]<<8) | buf[5];

    		lpf_red = 15*lpf_red;
    		lpf_red += v_red;
    		lpf_red /= 16;

    		lpf_ir = 15*lpf_ir;
    		lpf_ir += v_ir;
    		lpf_ir /= 16;

    		print_decimal(ts);
    		print_byte(' ');
    		//print_decimal(nsamples);
    		//print_byte(' ');
    		//print_decimal(fifo_write_ptr);
    		//print_byte(' ');
    		print_decimal(v_red);
    		print_byte(' ');
    		print_decimal(v_ir);
    		print_byte(' ');

    		print_decimal(v_red - lpf_red);
    		print_byte(' ');
    		print_decimal(v_ir - lpf_ir);
    		print_byte(' ');

    		print_decimal(npoll);
    		print_byte('\r');
    		print_byte('\n');
    		fifo_read_ptr++;
    		prev_ts = ts;
    		last_fifo_write_ptr = fifo_write_ptr;
    		npoll=0;
    	} else {
    		npoll++;
    	}
    }
    return 0 ;
}
