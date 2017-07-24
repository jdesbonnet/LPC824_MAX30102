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

#define VERSION "0.2.0"

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "printf.h"

//
// Hardware configuration
//

// Number of sensors
#define NSENSOR 2


//#define UART_BAUD_RATE 115200
//#define UART_BAUD_RATE 230400
#define UART_BAUD_RATE 460800

#define EOL "\r\n"

// Define function to pin mapping. Pin numbers here refer to PIO0_n
// and is not the same as a package pin number.
// Use PIO0_0 and PIO0_4 for UART RXD, TXD (same as ISP)
#define PIN_UART_RXD 0
#define PIN_UART_TXD 4
// General purpose debug pin
#define PIN_DEBUG 14

// Interrupt pins for the two sensors
#define PIN_INT0 13
#define PIN_INT1 1

// I2C bus pins for the two sensors
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


// To facilitate tfp_printf()
void myputc (void *p, char c) {
	Chip_UART_SendBlocking(LPC_USART0, &c, 1);
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

/**
 * Configure MAX30102 by writing to registers.
 *
 * @param sample_rate 0 = 50sps, 1=100sps, 2=200sps.
 */
setup_max30102 (LPC_I2C_T *i2c_bus, int sample_rate) {

	// Constrain sample_rate to 0..3
	sample_rate &= 0x3;

	// Reset
	hw_i2c_register_write(i2c_bus, 0x9, 1<<6);
	hw_i2c_register_write(i2c_bus, 0x9, 1<<6);

	// Interrupt Enable 1 Register. Set PPG_RDY_EN (data available in FIFO)
	hw_i2c_register_write(i2c_bus, 0x2, 1<<6);

	// FIFO configuration register
	// SMP_AVE: 16 samples averaged per FIFO sample
	// FIFO_ROLLOVER_EN=1
	//hw_i2c_register_write(0x8,  1<<4);
	hw_i2c_register_write(i2c_bus, 0x8, (0<<5) | 1<<4);

	// Mode Configuration Register
	// bit 7: SHDN
	// bit 6: RESET
	// bit 0:2: MODE: 2: HR (red only), 3: SPO2 (red+nir), 7: multi-led mode (red+nir)
	hw_i2c_register_write(i2c_bus, 0x9, 3);

	// SPO2 Configuration Register
	// bit 7: n/a
	// bit 5:6: SPO2_ADC_RGE[1:0] (full scale is 0: 2048nA, 1: 4096, 2: 8192nA, 3: 16384nA)
	// bit 2:4: SPO2_SR[2:0]: sample rate (0: 50, 1: 100, 2: 200, 3: 400, 4: 800... 7:3200)
	// bit 0:1: LED_PW[1:0]: pulse width (0: 69us, 1: 118us, 2:215us, 3:411us)
	// 18bits resolution only available with LED_PW=3
	hw_i2c_register_write(i2c_bus, 0xa,
			(3<<5)  // SPO2_ADC_RGE 2 = full scale 8192 nA (LSB size 31.25pA); 3 = 16384nA
			| (sample_rate<<2) // sample rate: 0 = 50sps; 1 = 100sps; 2 = 200sps
			| (3<<0) // LED_PW 3 = 411µs, ADC resolution 18 bits
	);

	// LED1 (red) power (0 = 0mA; 255 = 50mA)
	hw_i2c_register_write(i2c_bus, 0xc, 0xb0);

	// LED (IR) power
	hw_i2c_register_write(i2c_bus, 0xd, 0xa0);
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

void WDT_IRQHandler (void) {

	tfp_printf("#W%s",EOL);

	// Wait until all data sent on wire
	while ( ! (LPC_USART0->STAT & (1<<3)) );

	NVIC_SystemReset();
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
	setup_pin_for_interrupt(PIN_INT0, 7);
	setup_pin_for_interrupt(PIN_INT1, 6);

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

	init_printf(NULL,myputc);

	// Use SCT for hi-res timer.
	setup_sct_for_timer();


	//
	// Enable watchdog timer
	//

    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<17);

	// Power to WDT
	LPC_SYSCON->PDRUNCFG &= ~(0x1<<6);
	// Setup watchdog oscillator frequency
	// FREQSEL (bits 8:5) = 0x1 : 0.6MHz  +/- 40%
	// DIVSEL (bits 4:0) = 0x1F : divide by 64
	// Watchdog timer: ~ 10kHz
    LPC_SYSCON->WDTOSCCTRL = (0x1<<5) |0x1F;
    LPC_WWDT->TC = 4000;
    LPC_WWDT->MOD = (1<<0) // WDEN enable watchdog
    			| (1<<1); // WDRESET : enable watchdog to reset on timeout
    // Watchdog feed sequence
    LPC_WWDT->FEED = 0xAA;
    LPC_WWDT->FEED = 0x55;
    NVIC_EnableIRQ( (IRQn_Type) WDT_IRQn);


	//
	// Initialize GPIO
	//
	//Chip_GPIO_Init(LPC_GPIO_PORT);
	uint32_t clock_hz = Chip_Clock_GetSystemClockRate();


	hw_i2c_setup(LPC_I2C0);
	hw_i2c_setup(LPC_I2C1);

	setup_max30102(LPC_I2C0,1);
	setup_max30102(LPC_I2C1,1);

	uint32_t v_red,v_nir;

	int32_t lpf_red[NSENSOR] = {0,0};
	int32_t lpf_nir[NSENSOR] = {0,0};

	// Subtract DC component and output as extra column
	uint8_t output_ac_col = 0;

	// Include hi-res timestamp in output
	uint8_t output_hires_time = 0;

	uint32_t sample_counter=0;

	uint8_t buf[6];

	uint8_t fifo_read_ptr[NSENSOR];
	uint8_t fifo_write_ptr[NSENSOR];
	int32_t temperature[NSENSOR];

	uint32_t device_index;
	uint32_t checksum;

	LPC_I2C_T *device;

	LPC_I2C_T *device_i2c_bus[NSENSOR] = {LPC_I2C0, LPC_I2C1};

	int i;

	for (i = 0; i < NSENSOR; i++) {
		fifo_read_ptr[i] =  hw_i2c_register_read(device_i2c_bus[i], 0x6);
	}

	uint8_t last_fifo_write_ptr[NSENSOR]={0,0};

	int intreg;
	char c;

	// System booting and version
	tfp_printf("#S %s%s",VERSION,EOL);


	// What sensor hardware revision?
	tfp_printf("#V%s",EOL);
	for (i = 0; i < NSENSOR; i++) {
		tfp_printf(" %d",hw_i2c_register_read(device_i2c_bus[i],0xfe));
	}
	tfp_printf(EOL);

	// Note about datarates: LPC824 I2C0 can operate up to 1Mpbs, Other I2C
	// interfaces are limited to 400kbps. Each sample requires 2 register
	// reads and 6 bytes of sample data. 64bits x 100sps = 6.4kbps
    while(1) {

        // Watchdog feed sequence
        LPC_WWDT->FEED = 0xAA;
        LPC_WWDT->FEED = 0x55;

    	// UART simple commands. Single char. "0".."2" set sample speeds 50sps,100sps,200sps.2
    	if (LPC_USART0->STAT & 1) {
    		c = LPC_USART0->RXDATA;
    		switch (c) {
    		case '0':
    		case '1':
    		case '2':
    		{
    			setup_max30102(LPC_I2C0, c - '0');
    			setup_max30102(LPC_I2C1, c - '0');
    			tfp_printf("#%cEOL",c,EOL);
    			break;
    		}
    		case 'T':
    		{
    			output_hires_time = 1;
    			break;
    		}
    		case 't':
    		{
    			output_hires_time = 0;
    			break;
    		}
    		case 'A':
    		{
    			output_ac_col = 1;
    			break;
    		}
    		case 'a':
    		{
    			output_ac_col = 0;
    			break;
    		}
    		}
    	}


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


    		tfp_printf("$PPGV0 ");
            // Timestamp in microseconds (timer clocked by 30MHz clock).
    		tfp_printf("%x %x %x %x", device_index, v_red, v_nir, (data_timestamp[device_index]/30)&0xffffff);

    		if (output_ac_col) {
    			tfp_printf(" %d %d", v_red - lpf_red[device_index], v_nir - lpf_nir[device_index]);
    		}

    		tfp_printf(" *%x%s", (v_red+v_nir)&0xff, EOL);

    		fifo_read_ptr[device_index]++;
    		last_fifo_write_ptr[device_index] = fifo_write_ptr[device_index];

    		// Periodically query die temperature (required for SPO2 calculations)
    		if (sample_counter % 4096 == 0) {
    			for (i = 0; i < NSENSOR; i++) {
    				temperature[i] = -1;
        			hw_i2c_register_write(device_i2c_bus[i],0x21,1);
    			}
    			int tcount = 0;
    			while (tcount < NSENSOR) {
    				for (i = 0; i < NSENSOR; i++) {
    					if (temperature[i]==-1 && hw_i2c_register_read(device_i2c_bus[i],0x21)==0) {
    						temperature[i] = hw_i2c_register_read(device_i2c_bus[i],0x1f) << 4
    								| hw_i2c_register_read(device_i2c_bus[i],0x20);
    						tcount++;
    					}
    				}
    			}

    			tfp_printf("#T");
    			for (i = 0; i < NSENSOR; i++) {
    				tfp_printf(" %d", temperature[i]);
    			}
    			tfp_printf(EOL);
    		}
    		sample_counter++;
    	}

    }
    return 0 ;
}
