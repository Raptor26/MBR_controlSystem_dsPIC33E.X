/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef RBS_ROBOT_BALANCING_SYSTEM_H_
#define RBS_ROBOT_BALANCING_SYSTEM_H_


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
#include <math.h>
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "../inc/RPA_robot_pitch_angle.h"
#include "../../Lib_A_REGUL_regulators/Lib_A_REGUL_regulators.h"
#include "../../Lib_A_FILT_filters.c/Lib_A_FILT_filters.h"
#include "main.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
typedef enum
{
	RBS_LEFT_MOTOR = 0,
	RBS_RIGHT_MOTOR,

	RBS_MOTOR_NUMB,
} rbs_left_right_motors_e;

typedef struct
{
	/**
	    * @brief Целевая скорость, которую нужно достичь
	    */
	__PFPT__ target;

	/**
	 * @brief Текущая скорость
	 */
	__PFPT__ currSpeed;

	/**
	 * @brief Текущая скорость с применением фильтра
	 */
	__PFPT__ currSpeedFilt;

	/**
	 * @brief Комплементарный фильтр для фильтрации текущей скорости
	 */
	FILT_comp_filt_s compFilt_s;

	regul_pid_s piRegulator_s;
} rbs_speed_control_s;

typedef struct
{
	regul_pid_s pdForBalance_s;
	__PFPT__ motorControl_a[RBS_MOTOR_NUMB];
	__PFPT__ motorControl;
	__PFPT__ desiredAngle;

	/**
	 * @brief Структура для управления скорость движения робота
	 */
	rbs_speed_control_s speedControl_s;
} rbs_balancing_system_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
extern rbs_balancing_system_s RBS_balancingSystem_s;
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void
RBS_Init_BalancingSystem(
	rbs_balancing_system_s *p_s);

extern __PFPT__
RBS_GetControlForRobot(
	rbs_balancing_system_s *p_s,
	__PFPT__ pitchAngle,
	__PFPT__ pitchAngularSpeed);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* RBS_ROBOT_BALANCING_SYSTEM_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
