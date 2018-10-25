/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/LRMC_left_right_motor_control.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
vmcpc_f3m_package_s LRMC_leftRightMotorControlPack_s __attribute__((space(xmemory)));
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
static void
LRMC_Init_IOPorts(
	void);

static void
LRMC_Init_UART2_TransmitReceive(
	unsigned long fcy,
	unsigned long baudrate);

static void
LRMC_Init_DMA2_ForUart2Tramsmit(
	void);
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
void
LRMC_Init_UART_DMA_IOPins(
	unsigned long fcy,
	unsigned long baudrate)
{
	/* Инициализация портов ввода/вывода */
	LRMC_Init_IOPorts();

	/* Инициализация модуля USART */
	LRMC_Init_UART2_TransmitReceive(
		fcy,
		baudrate);

	/* Инициализация модуля DMA */
	LRMC_Init_DMA2_ForUart2Tramsmit();
}

void
LRMC_SendCmdForLeftRightMotors(
	vmcpc_f3m_package_s *p_s,
	__VMCPC_FPT__ leftMotor,
	__VMCPC_FPT__ rightMotor)
{
	/* Формирование пакета данных */
	VMCPC_F3M_SetControlPackage(
		p_s,
		rightMotor,
		leftMotor,
		(__VMCPC_FPT__) 0.0f);

	/* Старт передачи пакета данных */
	LRMC_Start_UART2DMATramsmit(
		(unsigned int*) p_s,
		(unsigned int) sizeof(vmcpc_f3m_package_s));
}

void
LRMC_Init_IOPorts(
	void)
{
	//  "UART 2 <RX>" - "RC13/PGED2/SOSCI/C3IN3-/RPI61" - "0111101"
	//  |Input Name     |   |Function Name|     |Register|      |Configuration Bits|
	//  |UART2 Receive  |   |U2RX         |     |RPINR19 |      |U2RXR 6:0         |
	ANSELCbits.ANSC13 = 0;
	TRISCbits.TRISC13 = 1;
	RPINR19bits.U2RXR = 0b0111101;

	/* FIXME удалить ANSELD = 0x0000 и проверить роботоспособность */
	ANSELD = 0x0000;
	//  "UART 2 <TX>" - "INT0/DMH/RP64/RD0"
	//  |Function|      |RPnR 5:0 |     |Output Name                 |
	//  |U2TX    |      |000011   |     |RPn tied to UART2 Transmit  |
	TRISDbits.TRISD0 = 0;
	RPOR0bits.RP64R = 0b000011;
}

void
LRMC_Init_UART2_TransmitReceive(
	unsigned long fcy,
	unsigned long baudrate)
{
//	unsigned int config1 =
//		UART_EN
//		& UART_IDLE_CON
//		& UART_IrDA_DISABLE
//		& UART_MODE_SIMPLEX
//		& UART_UEN_00
//		& UART_DIS_WAKE
//		& UART_DIS_LOOPBACK
//		& UART_DIS_ABAUD
//		& UART_UXRX_IDLE_ONE
//		& UART_BRGH_SIXTEEN
//		& UART_NO_PAR_8BIT
//		& UART_1STOPBIT,
//
//		config2 =
//			UART_INT_TX
//			& UART_IrDA_POL_INV_ZERO
//			& UART_SYNC_BREAK_DISABLED
//			& UART_TX_ENABLE
//			& UART_TX_BUF_NOT_FUL
//			& UART_INT_RX_CHAR
//			& UART_ADR_DETECT_DIS
//			& UART_RX_OVERRUN_CLEAR,
//
//			U_BRG =
//				((fcy / baudrate) / 16) - 1;
//
//	OpenUART2(
//		config1,
//		config2,
//		U_BRG);
//
//	ConfigIntUART2(
//		UART_RX_INT_EN
//		& UART_RX_INT_PR7
//		& UART_TX_INT_EN
//		& UART_TX_INT_PR7);

	PIC_Init_USART_2_1StopBit_8BitData_RxIntEnChar_TxIntEnChar(fcy, baudrate);
}

void
LRMC_Init_DMA2_ForUart2Tramsmit(
	void)
{
//	unsigned int config =
//		DMA2_MODULE_OFF
//		& DMA2_SIZE_BYTE
//		& DMA2_TO_PERIPHERAL
//		& DMA2_INTERRUPT_BLOCK
//		& DMA2_NORMAL
//		& DMA2_REGISTER_POST_INCREMENT
//		& DMA2_ONE_SHOT;
//	unsigned int irq_ = DMA2_AUTOMATIC;
//	unsigned int sta_address = (unsigned int)&LRMC_leftRightMotorControlPack_s;
//	unsigned int stb_address = (unsigned int)&LRMC_leftRightMotorControlPack_s;
//	unsigned int pad_address = (volatile unsigned int) &U2TXREG;
//	unsigned int count = 15u;
//
//	CloseDMA2();
//	OpenDMA2(
//		config,
//		irq_,
//		sta_address,
//		stb_address,
//		pad_address,
//		count);
//
//	ConfigIntDMA2(
//		DMA2_INT_PRI_5
//		& DMA2_INT_ENABLE);
//
//	IFS1bits.DMA2IF = 0;
//
//	/* UART2TX – UART2 Transmitter */
//	DMA0REQbits.IRQSEL = 0b00011111;
//
//	DMA2CONbits.CHEN = 0;
//
//	EnableIntDMA2;

	/* 00 = Register Indirect with Post-Increment mode */
	DMA2CONbits.AMODE = 0b00;

	/* 1 = Byte */
	DMA2CONbits.SIZE = 1;

	/* 01 = One-Shot, Ping-Pong modes disabled */
	DMA2CONbits.MODE = 0b01;

	/* Read from DPSRAM (or RAM) address, write to peripheral address */
	DMA2CONbits.DIR = 1;

	DMA2CNT = 0;

	/* UART2TX – UART2 Transmitter 00011111   */
	DMA2REQ = 0b00011111;

	/*  Peripheral Address Register bits */
	DMA2PAD = (volatile unsigned int) &U2TXREG;

	DMA2STAL = 0x0000;
	DMA2STAH = 0x0000;
	DMA2STBL = 0x0000;
	DMA2STBH = 0x0000;

	/* Clear DMA Interrupt Flag */
	IFS1bits.DMA2IF = 0;

	/* Enable DMA Interrupt */
	IEC1bits.DMA2IE = 1;

	DMA2CONbits.CHEN = 0;
}

void
LRMC_StartForce_UART2DMATransmit(
	unsigned int *pMemSrc,
	unsigned int cnt)
{
//	U2STAbits.UTXEN = 0;
	/* Отключение канала DMA */
	/* FIXME раскомментировать сторку ниже */
//	DMA2CONbits.CHEN = 0;

//	unsigned int trash = U2TXREG;
//	trash = U2TXREG;
//	trash = U2TXREG;
//	trash = U2TXREG;

	/* Сброс флага Overrun модуля UART */
//	U2STAbits.OERR = 0;

	/* Присваивание адреса в памяти */
	DMA2STAL = (unsigned int)pMemSrc;
//	DMA2STAH = (unsigned int)pMemSrc;

	/* Установка количества байт, которое необходимо передать */
	DMA2CNTbits.CNT = cnt - 1u;

	/* Старт канала DMA */
	DMA2CONbits.CHEN = 1;
	DMA2REQbits.FORCE = 1;
//	U2STAbits.UTXEN = 1;
}

void
LRMC_Start_UART2DMATramsmit(
	unsigned int *pMemSrc,
	unsigned int cnt)
{
	if (DMA2CONbits.CHEN == 0)
	{
		LRMC_StartForce_UART2DMATransmit(
			pMemSrc,
			cnt);
	}
}

void __attribute__ ((__interrupt__, auto_psv))
_U2TXInterrupt (void)
{
	/* Сброс флага прерывания */
	IFS1bits.U2TXIF = 0;
}

void __attribute__ ((__interrupt__, auto_psv))
_U2RXInterrupt (void)
{
	/* Сброс флага прерывания */
	IFS1bits.U2RXIF = 0;
}

void __attribute__ ((__interrupt__, auto_psv))
_DMA2Interrupt (void)
{
	/* Сброс флага прерывания */
	IFS1bits.DMA2IF = 0;

	/* Отключение канала DMA */
	DMA2CONbits.CHEN = 0;
//	U2STAbits.UTXEN = 0;
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
