/**
 * @file    %<%NAME%>%.%<%EXTENSION%>%
 * @author  %<%USER%>%
 * @version
 * @date    %<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/HPT_hard_prog_tact.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
ptwt_prog_tact_s HPT_hardProgTact_s;
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
static void
HPT_Init_TMRForProg_Tact(
	unsigned int cnt);
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
void
HPT_Init_TMR9ForProgTact_PTWTLibrary(
	unsigned int cnt)
{
	/* Инициализация структуры для тактирования программы */
	ptwt_prog_tact_init_struct_s progTactInit_s;
	progTactInit_s.pHardCnt			= (uint16_t*) &TMR9;
	progTactInit_s.progTactTime		= cnt;
	PTWT_Init_ProgTactStruct(
		&HPT_hardProgTact_s,
		&progTactInit_s);

	/* Инициализация аппаратного таймера */
	HPT_Init_TMRForProg_Tact(cnt);
}

void
HPT_Init_TMRForProg_Tact(
	unsigned int cnt)
{
	unsigned int config =
		T9_ON
		& T9_SOURCE_INT
		& T9_IDLE_CON
		& T9_PS_1_64
		& T9_GATE_OFF;

	OpenTimer9(
		config,
		cnt);

	ConfigIntTimer9(
		T9_INT_PRIOR_1
		& T9_INT_ON);
}

void  __attribute__ ((__interrupt__, auto_psv))
_T9Interrupt (void)
{
	IFS3bits.T9IF = 0;  // Clear Timer9 Interrupt Flag```````````````````````````````````

	PTWT_ProgTactISR(
		&HPT_hardProgTact_s);
//	HPT_hardProgTact_s.enable_flag++;
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
