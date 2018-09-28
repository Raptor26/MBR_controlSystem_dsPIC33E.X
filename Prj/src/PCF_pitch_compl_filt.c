/**
 * @file    PCF_pitch_compl_filt.c
 * @author  Kuroha
 * @version
 * @date    28 сентября 2018 г., 12:55
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/PCF_pitch_compl_filt.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
float
PCF_GetPitchAngle(
	float accX,
	float accZ,
	float gyrY,
	float oldAngle,
	float compFiltCoeff,
	float dT)
{
	/* Получить угол наклона по показаниям акселеромтера */
	float pitchByAcc = atan2(accX, accZ);

	/* Найти прирщение угла наколна за промежуток времени dT */
	float deltaPitch = gyrY * dT;

	return (((oldAngle + deltaPitch) * compFiltCoeff) + (pitchByAcc * (1 - compFiltCoeff)));
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
