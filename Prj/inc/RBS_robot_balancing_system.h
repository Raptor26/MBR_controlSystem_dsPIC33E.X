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
#include "../inc/CMP_control_message_parser.h"
#include "../inc/RPA_robot_pitch_angle.h"
#include "../../Lib_A_REGUL_regulators/Lib_A_REGUL_regulators.h"
#include "../../Lib_A_FILT_filters.c/Lib_A_FILT_filters.h"
#include "main.h"
#include "../../Lib_A_REGUL_regulators/Lib_A_REGUL_IBSC_regulator.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
#if __PFPT_CHAR__  == 'f'
#define __RBS_sin(x)		sinf(x)
#define __RBS_pow(x,y)    powf(x,y)
#define __RBS_fabs(x)		fabsf(x)

#elif __PFPT_CHAR__  == 'd'
#define __RBS_sin(x)		sin(x)
#define __RBS_pow(x,y)    pow(x,y)
#define __RBS_fabs(x)		fabs(x)
#endif
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
#define RBS_90DEG_IN_RAD      ((__PFPT__)((3.14159265)*0.5))
#define RBS_45DEG_IN_RAD      ((RBS_90DEG_IN_RAD * ((__PFPT__)0.5)))
#define RBS_30DEG_IN_RAD      ((RBS_90DEG_IN_RAD) * ((__PFPT__)0.33333333))
#define RBS_MAX_ANGLE_RAD     (RBS_45DEG_IN_RAD * 1.2)
typedef enum
{
	RBS_LEFT_MOTOR = 0,
	RBS_RIGHT_MOTOR,

	RBS_MOTOR_NUMB,
} rbs_left_right_motors_e;

typedef struct
{    
	/**
	 * @brief Текущая скорость
	 */
	__PFPT__ currSpeed;

	/**
	 * @brief Текущая скорость с применением фильтра
	 */
	__PFPT__ currSpeedFilt;
    
    __PFPT__ balancedAngle;
    
    filt_complementary_s compFilt_balancedAngle_s;
    

	/**
	 * @brief Комплементарный фильтр для фильтрации текущей скорости
	 */
	filt_complementary_s compFilt_s;

	regul_pid_s piRegulator_s;
    
    regul_pid_s dRegulator_s;
    
	/**
	 * @brief Структура с целевой скоростью и поворотом
	 */
	cmp_control_data_s control_data_s;
} rbs_speed_control_s;

typedef struct
{
	regul_pid_s pdForBalance_s;
    
    regul_ibsc_s ibscForBalance_s;
    
	__PFPT__ motorControl_a[RBS_MOTOR_NUMB];
	__PFPT__ motorControl;
	__PFPT__ desiredAngle;

	/**
	 * @brief Структура для управления скорость движения робота
	 */
	rbs_speed_control_s speedControl_s;
    
    size_t startSystem_flag;
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
