/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/configuration_bits.h"
#include "../inc/main.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
float acc_a[IISMPU_VECT_SIZE];
float gyr_a[IISMPU_VECT_SIZE];
float mpuTemperature;

char testMessage_a[] = "Hello World.\n";
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/
char receiveTestMessage[4];
/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
VTMR_tmr_s compFiltRuntime_s;
size_t dmaReceiveEn_flag = 1;
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
static void
InitAllPeriphAndModules(
	void);
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
int main(
	void)
{
	/* Инициализация всей периферии и программных модулей находится в
	 * функции InitAllPeriphAndModules() */
	InitAllPeriphAndModules();

	/* Loop */
	while (1)
	{
		PTWT_ProgTactStartLoop(
			&HPT_hardProgTact_s);

		/* Опрос инерциального датчика и копирование его показаний в
		 * массивы */
		IISMPU_GetAccGyrTemperature(
			&acc_a[0],
			&gyr_a[0],
			&mpuTemperature);

		/* Определение угла наклона балансирующего робота по тангажу */
		VTMR_StartTimer(
			&compFiltRuntime_s);

		__PFPT__ robotPitchAngle =
			RPA_GetPitchAngle(
				&gyr_a[IISMPU_PITCH],
				acc_a[IISMPU_ROLL],
				acc_a[IISMPU_YAW]);

		VTMR_GetTimerValue(
			&compFiltRuntime_s);

		__PFPT__ leftRightMotorControl =
			RBS_GetMotorControlForBalancingRobot(
				&RBS_balancingSystem_s,
				robotPitchAngle,
				gyr_a[IISMPU_PITCH]);

		if (DMA2CONbits.CHEN == 0)
		{
			LRMC_SendCmdForLeftRightMotors(
				&LRMC_leftRightMotorControlPack_s,
				(__VMCPC_FPT__) RBS_balancingSystem_s.motorControl_a[RBS_LEFT_MOTOR],
				(__VMCPC_FPT__) RBS_balancingSystem_s.motorControl_a[RBS_RIGHT_MOTOR]);
		}

        if (dmaReceiveEn_flag == 1)
        {
            dmaReceiveEn_flag = 0;
            memset((void*) receiveTestMessage, 'g', 4);
            UDI_StartForceUart3_DMA4_Receiver((unsigned int*)receiveTestMessage, 3u);
        }
		/* ################ Отладочная информация ####################### */
		/* Формирование отладочного пакета данных */
//		UDI_GetAndSendDebugPackForSerialPlot(
//			&UDI_serialPlotDataPackage_s);
        
        UDI_StartUart3_DMA3_Transmit((unsigned int*)testMessage_a, strlen(testMessage_a));
        
        if (DMA4CONbits.CHEN == 0)
        {
//            UDI_StartForceUart3_DMA4_Receiver((unsigned int*)receiveTestMessage, 15);
        }
		/* ############################################################## */

		PTWT_ProgTactEndLoop(
			&HPT_hardProgTact_s);
		/* Здесь не должно быть НИЧЕГО!!! */
	}
	return (1);
}

void
InitAllPeriphAndModules(
	void)
{
	/* Запрет глобальных прерываний */
	_GIE = 0;

	/* Отключение вложенных прерываний */
	_NSTDIS = 1;

	/*=== |Begin| --> Секция - "Конфигурирование периферии микроконтроллера" =*/
	/* Инициализация тактового генератора */
#if defined (__USE_FRC_FOR_FCY__)
	PIC_Init_Oscillator_FRC_8MHz_FOSC_128MHz_FCY_64MIPS();
#elif defined (__USE_HS_16_MHz_FOR_FCY__)
	PIC_Init_Oscillator_HS_16MHz_FOSC_128MHz_FCY_64MIPS();
#else
#error "Please, set source for system clock"
#endif

	/* Инициализация светодиодов платы */
	BLEDS_Init_AllLeds();

	/* Инициализация UART модуля для передачи отладочной информации */
	UDI_Init_All_UART3_RxTx_With_DMA_Tx(
		(unsigned int long) FCY,
		(unsigned int long) 115200UL);

	/* Инициализация аппаратного таймера для тактирования цикла while(1) */
	HPT_Init_TMR9ForProgTact_PTWTLibrary(
		__HARD_PROG_TACT_IN_US__);

	/* Инициализация аппаратных таймеров для подключения в ним виртуальных
	 * таймеров */
	MC32_Init_32bitsCntForVirtTimers();

	/* Инициализация всей периферии для работы с внутренним инерциальным датчиком */
	IISMPU_Init_AllPeriphForInternalMPU6000();

	/* Инициализация UART2 для передачи команд контроллерам электродвигателей */
	LRMC_Init_UART_DMA_IOPins(
		(unsigned int long) FCY,
		(unsigned int long) 660000UL);
	/*=== |End  | <-- Секция - "Конфигурирование периферии микроконтроллера" =*/

	VTMR_InitTimerStruct(
		&compFiltRuntime_s,
		(uint16_t*) &TMR7,
		(uint16_t*) &TMR6);

	RPA_Init_DataForCalcPitchAngle();

	RBS_Init_BalancingSystem(
		&RBS_balancingSystem_s);
	
	/* Разрешение глобальных прерываний */
	_GIE = 1;
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
