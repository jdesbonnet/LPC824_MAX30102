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
	Chip_GPIO_Init(LPC_GPIO_PORT);

/*
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, PIN_I2C_SCL);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, PIN_I2C_SDA);

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SCL, false);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SCL, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SCL, false);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SCL, true);

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SDA, false);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SDA, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SDA, false);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_I2C_SDA, true);
*/

	// Experimental I2C MAX30102
	//i2c_setup(PIN_I2C_SCL, PIN_I2C_SDA);
	hw_i2c_setup();

	/*
	int rev,id,val0,val1;
	rev = max30102_register_read(0xfe);
	id = max30102_register_read(0xff);
	rev = max30102_register_read(0xfe);
	val0 = max30102_register_read(0x0a);
	max30102_register_write(0xa, 0xFD);
	val1 = max30102_register_read(0x0a);
*/

	hw_i2c_read(0xAE,0xFE,1,1);


    while(1) {
        i++ ;
    }
    return 0 ;
}
