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



void hw_i2c_read(int i2c_addr, int reg_addr, int len, int d) {

	I2CM_XFER_T  i2cmXferRec;

	uint8_t sendData[4] = {0xff,0,0,0};
	uint8_t recvData[4] = {0,0,0,0};


	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = 0xAE >> 1;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = 1;
	i2cmXferRec.rxSz = 1;
	i2cmXferRec.txBuff = &sendData[0];
	i2cmXferRec.rxBuff = &recvData[0];

	Chip_I2CM_XferBlocking(LPC_I2C, &i2cmXferRec);

	while (1) {

	}
}

#ifdef FALSE
void i2c_hw_write(int i2c_addr, int reg_addr, int len, int d) {
	uint8_t recvData[10];
	I2C_PARAM_T param;
	I2C_RESULT_T result;

	recvData[0] = i2c_addr;
	recvData[1] = reg_addr;

	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 (read) - ack
	   value 2 read) - ack - stop */
	param.num_bytes_send    = 0;
	param.num_bytes_rec     = 2;
	param.buffer_ptr_rec    = &recvData[0];
	param.stop_flag         = 1;


	/* Do master read transfer */
	int error_code = LPC_I2CD_API->i2c_master_receive_poll(i2cHandleMaster, &param, &result);
}
#endif

