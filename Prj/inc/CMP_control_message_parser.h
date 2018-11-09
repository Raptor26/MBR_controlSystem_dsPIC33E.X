/**
 * @file   	CMP_control_message_parser.h
 * @author 	Kuroha
 * @version
 * @date 	25 октября 2018 г., 12:47
 * @brief
 */


#ifndef CMP_CONTROL_MESSAGE_PARSER_H_
#define CMP_CONTROL_MESSAGE_PARSER_H_


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "../../Lib_A_VTMR_virtual_timers/Lib_A_VTMR_virtual_timers.h"
#include "../inc/MC32_hardware_counter_32.h"
#include "../../Lib_A_FILT_filters.c/Lib_A_FILT_filters.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
typedef enum
{
	CMP_MESSAGE_INVALID = 0,
	CMP_MESSAGE_VALID,
} cmp_message_status_e;

typedef struct
{
	/**
	 * @brief Целевая скорость, которую нужно достичь
	 */
	__PFPT__ targetSpeed;

	/**
	 * @brief Целевой поворот
	 */
	__PFPT__ targetRotation;

	__PFPT__ targetMaxVal;

	/**
	 * @brief Максимальное значение, которое регулирует скорость и поворот
	 */
	int16_t maxControlValue;

	__PFPT__  discreteInc;

	/**
	 * @brief Нулевое значение (равно половине от максимального значения)
	 */
	int16_t zeroValue;

	VTMR_tmr_s virtTmr;

	uint32_t maxTimeout;

	filt_complementary_s filtForTargetSpeed_s,
						 filtForTargetRotation_s;
} cmp_control_data_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
extern size_t CMP_receiveMessage_flag;
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void
CMP_init_struct(
	cmp_control_data_s *data);

extern cmp_message_status_e
CMP_parse_message(
	cmp_control_data_s *data,
	char *controlCmd);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* CMP_CONTROL_MESSAGE_PARSER_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
