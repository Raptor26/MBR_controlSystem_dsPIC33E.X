/**
 * @file    IISMPU_internal_inert_sens_mpu.
 * @author  Kuroha
 * @version
 * @date    28 сентября 2018 г., 10:53
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/IISMPU_internal_inert_sens_mpu.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
mpu60x0_spi_s IISMPU_SPIFnc_s;
mpu60x0_lsb_s IISMPU_LSB_s;
mpu60x0_data_s IISMPU_data_s;
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
void
IISMPU_Init_SPI(
	void);

void
IISMPU_Init_IO_PortsForSPI(
	void);

void
IISMPU_Init_MPU6000(
	void);
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
void
IISMPU_Init_AllPeriphForInternalMPU6000(
	void)
{
	IISMPU_Init_IO_PortsForSPI();
	IISMPU_Init_SPI();
	IISMPU_Init_MPU6000();
}

void
IISMPU_GetAccGyrTemperature(
	float *pAcc,
	float *pGyr,
	float *pTemperature)
{
	/* Опрос инерциального датчика */
	MPU60x0_SPI_GetAllNormData(
		&IISMPU_SPIFnc_s,
		&IISMPU_data_s,
		&IISMPU_LSB_s);

	/* Приведение к NED системе координат */
	IISMPU_SetNEDCoordinateSystem(&IISMPU_data_s.accelArr[MPU60x0_X]);
	IISMPU_SetNEDCoordinateSystem(&IISMPU_data_s.gyrArr[MPU60x0_X]);

	/* Копирование показаний акселерометра в массив */
	memcpy(
		(void*)pAcc,
		(void*)&IISMPU_data_s.accelArr[MPU60x0_X],
		sizeof(float) * IISMPU_VECT_SIZE);

	/* Копирование показаний гироскопа в массив */
	memcpy(
		(void*)pGyr,
		(void*)&IISMPU_data_s.gyrArr[MPU60x0_X],
		sizeof(float) * IISMPU_VECT_SIZE);

	/* Копирование температуры сенсора */
	*pTemperature = IISMPU_data_s.temper;
}

/**
 * @brief	Функция выполняет приведение системы координат внутреннего
 *			инерциального датчика к NED системе координат
 * @param[in,out]	pData[3]:	Указатель на нулевой элемент массива, в
 *								котором содержатся показания датчика по 3-м осям
 * @return	None
 */
void
IISMPU_SetNEDCoordinateSystem(
	float pData[])
{
	pData[0] = -pData[0];
}

/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
void
IISMPU_Init_SPI(
	void)
{
	PIC_Init_SPI_1_PriPRES_64_1_SecPRES_1_1_IntDis_8bits();
}

void
IISMPU_Init_IO_PortsForSPI(
	void)
{
	/* Конфигурирование  CS RB5/AN5/C1IN1+/VBUSON/VBUSST/RPI37 */
	// <Chip Select> for <MPU6000> - "RB5/AN5/C1IN1+/VBUSON/VBUSST/RPI37";
	TRISBbits.TRISB5 = 0;
	LATBbits.LATB5 = 1;

	/* Конфигурирование  MISO */
	//  "MISO_SPI1" - "PMCS1/RPI75/RD11" - "1001011"
	//  |Input Name     |   |Function Name|     |Register|      |Configuration Bits|
	//  |SPI1 Data Input|   |SDI1         |     |RPINR20 |      |SDI1R 6:0         |
	TRISEbits.TRISE1 = 1;
	RPINR20bits.SDI1R = 0b1001011;

	/* Конфигурирование  MOSI */
	//  "MOSI_SPI1" - "SDA2/PMA9/RP100/RF4"
	//  |Function|      |RPnR 5:0 |     |Output Name                 |
	//  |SDO1    |      |000101   |     |RPn tied to SPI1 Data Output|
	TRISFbits.TRISF4 = 0;
	RPOR9bits.RP100R = 0b000101;

	/* Конфигурирование  CLC */
	//  "CLC_SPI1" - "SCL2/PMA8/RP101/RF5"
	//  |Function|      |RPnR 5:0 |     |Output Name                  |
	//  |SCK1    |      |000110   |     |RPn tied to SPI1 Clock Output|
	TRISFbits.TRISF5 = 0;
	RPOR9bits.RP101R = 0b000110;
}

void
IIMPU_SPI_CS_On(void)
{
	/* RB5/AN5/C1IN1+/VBUSON/VBUSST/RPI37 */
	LATBbits.LATB5 = 0;
}

void
IIMPU_SPI_Cs_Off(void)
{
	/* RB5/AN5/C1IN1+/VBUSON/VBUSST/RPI37 */
	LATBbits.LATB5 = 1;
}

void
IIMPU_Delay_1ms(void)
{
	/* Задержка в 1 мс */
//    __delay_ms(1);
}

void
IISMPU_Init_MPU6000(
	void)
{
	IISMPU_SPIFnc_s.CS_Off = IIMPU_SPI_Cs_Off;
	IISMPU_SPIFnc_s.CS_On = IIMPU_SPI_CS_On;
	IISMPU_SPIFnc_s.Delay_1_us = IIMPU_Delay_1ms;
	IISMPU_SPIFnc_s.Receive_8bits = PIC_SPI1_Maste_Receiver_8bits;
	IISMPU_SPIFnc_s.Transmit_8bits = PIC_SPI1_Master_Transmit_8bits;

	mpu60x0_regs_s mpu6000ConfigRegs_s;
	mpu6000ConfigRegs_s.accelConf_28 = MPU60x0_BIT_AFS_SEL_4;
	mpu6000ConfigRegs_s.dlpf_26 = MPU60x0_BIT_DLPF_CFG_256;
	mpu6000ConfigRegs_s.fifo_En_35 = 0x00;
	mpu6000ConfigRegs_s.gyroConf_27 = MPU60x0_BIT_FS_SEL_250;
	mpu6000ConfigRegs_s.int_En_56 = 0x00;
	mpu6000ConfigRegs_s.int_pin_55 = 0x00;
	mpu6000ConfigRegs_s.int_status_58 = 0x00;
	mpu6000ConfigRegs_s.pwr_managment_107 = MPU60x0_BIT_CLKSEL_PLL_GYRO_X;

	IISMPU_LSB_s =
		MPU60x0_SPI_Config(
			&IISMPU_SPIFnc_s,
			&mpu6000ConfigRegs_s);
}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
