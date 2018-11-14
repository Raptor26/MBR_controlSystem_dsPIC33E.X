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
char debugControlCmd[50];
float acc_a[IISMPU_VECT_SIZE];
float gyr_a[IISMPU_VECT_SIZE];
float mpuTemperature;
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
char recievedControlCmd[14];
unsigned int controlCmdSize = sizeof (recievedControlCmd);
VTMR_tmr_s compFiltRuntime_s;
cmp_control_data_s controlRobot_s;
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
				(__PFPT__*) &gyr_a[IISMPU_PITCH],
				(__PFPT__) acc_a[IISMPU_ROLL],
				(__PFPT__) acc_a[IISMPU_PITCH],
				(__PFPT__) acc_a[IISMPU_YAW]);
		VTMR_GetTimerValue(
			&compFiltRuntime_s);

		/* ================================================================== */
		/* Если с момента обработки крайнего пакета данных для управления
		 * прошло времени больше чем величина "TimeOut" */
		if ((VTMR_GetTimerValue(
					&RBS_balancingSystem_s.speedControl_s.control_data_s.virtTmr))
				>= ((uint32_t)RBS_balancingSystem_s.speedControl_s.control_data_s.maxTimeout))
		{
			/* Сброс заданной скорости в нуль */
			RBS_balancingSystem_s.speedControl_s.control_data_s.targetSpeed =
				/* Комплементарный фильтр (используется для внесения задержки сигнала )*/
				FILT_Complementary_fpt(
					&RBS_balancingSystem_s.speedControl_s.control_data_s.filtForTargetSpeed_s,
					(__PFPT__) 0.0);

			/* Сброс заданной скорости вращения в нуль */
			RBS_balancingSystem_s.speedControl_s.control_data_s.targetRotation =
				/* Комплементарный фильтр (используется для внесения задержки сигнала )*/
				FILT_Complementary_fpt(
					&RBS_balancingSystem_s.speedControl_s.control_data_s.filtForTargetRotation_s,
					(__PFPT__) 0.0);
		}
		/* ================================================================== */

		/* Функция для получения управляющих воздействия для удержания робота 
		 * в заданном положении */
		__PFPT__ leftRightMotorControl =
			RBS_GetControlForRobot(
				&RBS_balancingSystem_s,
				robotPitchAngle,
				gyr_a[IISMPU_PITCH]);

		if (DMA2CONbits.CHEN == 0)
		{
			/* Формирование и запуск передачи пакета данных для управления
			 * электродвигателями */
			LRMC_SendCmdForLeftRightMotors(
				&LRMC_leftRightMotorControlPack_s,
				(__VMCPC_F3M_FPT__) RBS_balancingSystem_s.motorControl_a[RBS_LEFT_MOTOR],
				(__VMCPC_F3M_FPT__) RBS_balancingSystem_s.motorControl_a[RBS_RIGHT_MOTOR]);
		}

		if (CMP_receiveMessage_flag != 0 && DMA4CONbits.CHEN == 0)
		{
			CMP_receiveMessage_flag = 0;

			/* Обработать принятый пакет */
			cmp_message_status_e messageStatus_e =
				CMP_parse_message(
					&RBS_balancingSystem_s.speedControl_s.control_data_s,
					recievedControlCmd);

			/* Если принятый пакет данных валиден */
			if (messageStatus_e == CMP_MESSAGE_VALID)
			{
				/* Перезапуск виртуального таймера */
				VTMR_StartTimer(
					&RBS_balancingSystem_s.speedControl_s.control_data_s.virtTmr);
			}

			/* Забить память нулями */
			memset((void*) recievedControlCmd, '\0', controlCmdSize);

			// int bytesCnt =
			// 	sprintf(
			// 		debugControlCmd,
			// 		"speed = %f\r\ndirect = %f",
			// 		RBS_balancingSystem_s.speedControl_s.control_data_s.targetSpeed,
			// 		RBS_balancingSystem_s.speedControl_s.control_data_s.targetRotation);
		}

		/* ################ Отладочная информация ####################### */
		/* Формирование отладочного пакета данных */
//		UDI_GetAndSendDebugPackForSerialPlot(
//			&UDI_serialPlotDataPackage_s);
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

	/* Задержка чтобы успел проинициализироваться контроллер
	 * векторного управления */
	__delay_ms(100u);

	/* Инициализация светодиодов платы */
	BLEDS_Init_AllLeds();

	/* Инициализация UART модуля для передачи отладочной информации */
	UDI_Init_All_UART3_RxTx_With_DMA_Tx(
		(unsigned int long) FCY,
		(unsigned int long) 9600UL);

//    Конфигурация Bluetooth модуля:
//    AT+NAME=NCFU
//    AT+PSWD="NCFU9999"
//    AT+UART=460800,0,0

	UDI_StartForceUart3_DMA4_Receiver(
		(unsigned int*)recievedControlCmd,
		controlCmdSize);

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
		(uint16_t*) &HC32_UPPER_CNT,
		(uint16_t*) &HC32_LOWER_CNT);

	RPA_Init_DataForCalcPitchAngle();

	CMP_init_struct(
		&RBS_balancingSystem_s.speedControl_s.control_data_s);

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
