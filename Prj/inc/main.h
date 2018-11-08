/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef MAIN_H_
#define MAIN_H_

#define FCY                             64000000UL
/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
#include <xc.h>
#include <libpic30.h>
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "../inc/BLEDS_board_leds.h"
#include "../../Lib_H_PIC_oscillators/Lib_H_PIC_oscillators.h"
#include "../inc/UDI_uart_debug_information.h"
#include "../inc/HPT_hard_prog_tact.h"
#include "../inc/MC32_hardware_counter_32.h"
#include "../inc/IISMPU_internal_inert_sens_mpu.h"
#include "../../Lib_A_PCF_pitch_comp_filt/Lib_A_PCF_pitch_comp_filt.h"
#include "../../Lib_A_VTMR_virtual_timers/Lib_A_VTMR_virtual_timers.h"
#include "../inc/UDI_uart_debug_information.h"
#include "../inc/LRMC_left_right_motor_control.h"
#include "../inc/RPA_robot_pitch_angle.h"
#include "../inc/RBS_robot_balancing_system.h"
#include "../inc/CMP_control_message_parser.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
#if (__PFPT_CHAR__  == 'f') && (!defined __PFPT__)
#error "Define __PFPT__ = float"
#elif (__PFPT_CHAR__  == 'd') && (!defined __PFPT__)
#error "Define __PFPT__ = double"
#endif

#if (!defined __PFPT_CHAR__) && (!defined __PFPT__)
#error "Define __PFPT_CHAR__ = 'f' and __PFPT__ = float or __PFPT_CHAR__ = 'd' and __PFPT__ = double"
#endif

#define INTEGRATE_PERIOD_IN_SEC     ((__PFPT__)((__PFPT__)__HARD_PROG_TACT_IN_US__ / 1000000.0f))
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
extern char debugControlCmd[50];
extern float acc_a[3];
extern float gyr_a[3];
extern float mpuTemperature;
extern unsigned int controlCmdSize;
extern char recievedControlCmd[14];
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* MAIN_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
