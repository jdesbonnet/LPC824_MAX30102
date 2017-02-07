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

	hw_i2c_setup();

	// Reset
	hw_i2c_register_write(0x9, 1<<6);
	i2c_delay();

	//hw_i2c_register_write(0x8,  1<<4);
	hw_i2c_register_write(0x8, (8<<5) | 1<<4);

	i2c_delay();

	// Mode
	hw_i2c_register_write(0x9, 3);
	i2c_delay();

	hw_i2c_register_write(0xa, (2<<5) | (2<<2) | (3<<0) );
	i2c_delay();

	// LED power
	hw_i2c_register_write(0xc, 0x20);
	i2c_delay();

	// LED power
	hw_i2c_register_write(0xd, 0x20);
	i2c_delay();

	uint8_t v0,v1,v2;
	uint8_t fifo_read_ptr =  hw_i2c_register_read(0x6);
	uint8_t fifo_write_ptr;
	uint8_t nsamples;
	uint32_t vred,vir;

	uint8_t buf[6];

    while(1) {
    	// Get FIFO write pointer
    	fifo_write_ptr = hw_i2c_register_read(0x4);
    	i2c_delay();

    	nsamples = fifo_write_ptr - fifo_read_ptr;

    	for (i = 0; i < nsamples; i++) {

    		hw_i2c_fifo_read(buf,6);
    		/*
    		v0 = hw_i2c_register_read(0x7);
    		v1 = hw_i2c_register_read(0x7);
    		v2 = hw_i2c_register_read(0x7);
    		vred = (v0 << 16) | (v1 << 8) | v2;
    		v0 = hw_i2c_register_read(0x7);
    		v1 = hw_i2c_register_read(0x7);
    		v2 = hw_i2c_register_read(0x7);
    		vir = (v0 << 16) | (v1 << 8) | v2;
			*/
    		vred = (buf[0]<<16) | (buf[1]<<8) | buf[2];
    		vir = (buf[3]<<16) | (buf[4]<<8) | buf[5];
    		print_decimal(vred);
    		print_byte(' ');
    		print_decimal(vir);
    		print_byte('\r');
    		print_byte('\n');
    		fifo_read_ptr++;
    	}
    }
    return 0 ;
}
