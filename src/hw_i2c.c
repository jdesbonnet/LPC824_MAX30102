#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif


/* 100kbps I2C bit-rate */
#define I2C_BITRATE             (10000)

#define I2C_CLK_DIVIDER         (40)

void hw_i2c_setup (void) {
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Enable I2C clock and reset I2C peripheral - the boot ROM does not
	   do this */
	Chip_I2C_Init(LPC_I2C);

	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(LPC_I2C, I2C_CLK_DIVIDER);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C, I2C_BITRATE);

	/* Enable Master Mode */
	Chip_I2CM_Enable(LPC_I2C);

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

void hw_i2c_register_write(int reg_addr, int value) {

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

	Chip_I2CM_XferBlocking(LPC_I2C, &i2cmXferRec);

}
