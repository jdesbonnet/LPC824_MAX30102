#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif


/* 100kbps I2C bit-rate */
#define I2C_BITRATE             (40000)

#define I2C_CLK_DIVIDER         (8)

/**
 * Enable I2C. LPC824 has one fast (1Mbps) I2C on fixed pins (I2C0) and 3 others which
 * can be assigned to any GPIO pin, but limited to 400kbps max.
 */
void hw_i2c_setup (LPC_I2C_T *i2c) {

	// Enable I2C0 fixed location pins and second I2C1 on movable pins.
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	// I2C0 (fast fixed I2C) for first sensor
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);

	// I2C1 for second sensor
	Chip_SWM_MovablePinAssign(SWM_I2C1_SCL_IO, 8);
	Chip_SWM_MovablePinAssign(SWM_I2C1_SCL_IO, 9);

	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	// Enable I2C clock and reset I2C peripheral
	Chip_I2C_Init(i2c);
	// Setup clock rate for I2C
	Chip_I2C_SetClockDiv(i2c, I2C_CLK_DIVIDER);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(i2c, I2C_BITRATE);

	/* Enable Master Mode */
	Chip_I2CM_Enable(i2c);

}



int hw_i2c_register_read(int reg_addr) {

	I2CM_XFER_T  i2cmXferRec;

	uint8_t sendData[4] = {reg_addr,0,0,0};
	uint8_t recvData[4] = {0,0,0,0};


	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = 0xAE >> 1;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = 1;
	i2cmXferRec.rxSz = 1;
	i2cmXferRec.txBuff = &sendData[0];
	i2cmXferRec.rxBuff = &recvData[0];

	Chip_I2CM_XferBlocking(LPC_I2C, &i2cmXferRec);

	return recvData[0];
}

int hw_i2c_fifo_read(uint8_t *buf, int len) {

	I2CM_XFER_T  i2cmXferRec;

	uint8_t sendData[4] = {0x7,0,0,0};

	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = 0xAE >> 1;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = 1;
	i2cmXferRec.rxSz = len;
	i2cmXferRec.txBuff = &sendData[0];
	i2cmXferRec.rxBuff = buf;

	Chip_I2CM_XferBlocking(LPC_I2C, &i2cmXferRec);
}
void hw_i2c_register_write(LPC_I2C_T *i2c, int reg_addr, int value) {

	I2CM_XFER_T  i2cmXferRec;

	uint8_t sendData[4] = {reg_addr,value,0,0};
	uint8_t recvData[4] = {0,0,0,0};


	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = 0xAE >> 1;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = 2;
	i2cmXferRec.rxSz = 0;
	i2cmXferRec.txBuff = &sendData[0];
	i2cmXferRec.rxBuff = &recvData[0];

	Chip_I2CM_XferBlocking(i2c, &i2cmXferRec);

	i2c_delay();

}
