/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/RPA_robot_pitch_angle.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
pcf_data_for_pitch_s RPA_copmFiltDataForPitch_s;
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
__PFPT__
RPA_GetPitchAngle(
	__PFPT__ *pGyrPitch,
	__PFPT__ accX,
	__PFPT__ accY,
	__PFPT__ accZ)
{
	return (PCF_GetPitchByCompFilt(
				&RPA_copmFiltDataForPitch_s,
				(__PCF_FPT__*)pGyrPitch,
				(__PCF_FPT__)accX,
				(__PCF_FPT__)accY,
				(__PCF_FPT__)accZ));
}

void
RPA_Init_DataForCalcPitchAngle(
	void)
{
	/* Инициализация констант для вычисления угла наклона*/
	pcf_data_for_pitch_init_struct_s init_s;
	PCF_CompFilt_StructInit(
		&init_s);
	init_s.compFiltCoeff		= (__PCF_FPT__) 0.9996f;
	init_s.integralCoeff		= (__PCF_FPT__) 0.00002f;
	init_s.dT					= (__PCF_FPT__) INTEGRATE_PERIOD_IN_SEC;
	init_s.accNormWindow		= (__PCF_FPT__) 0.15;
	init_s.compFiltValForAcc	= (__PCF_FPT__) 0.0;
	init_s.integralErrorSaturation = (__PCF_FPT__) 0.15;
	PCF_Init_CompFilt(
		&RPA_copmFiltDataForPitch_s,
		&init_s);
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
