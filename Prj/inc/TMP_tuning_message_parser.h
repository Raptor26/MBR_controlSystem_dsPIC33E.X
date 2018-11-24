/**
 * @file    %<%NAME%>%.%<%EXTENSION%>%
 * @author  %<%USER%>%
 * @version
 * @date    %<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef TMP_TUNING_MESSAGE_PARSER_H_
#define TMP_TUNING_MESSAGE_PARSER_H_


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
#include "../inc/main.h"
#include "../../Lib_A_REGUL_regulators/Lib_A_REGUL_regulators.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
typedef struct {
	uint8_t header_byte;

	uint8_t id_byte;

	uint8_t class_byte;

	uint16_t payload;

	__REGUL_FPT__ new_kP;

	__REGUL_FPT__ new_kI;

	__REGUL_FPT__ new_kD;

	uint8_t crc;
}
#if defined (__GNUC__)
__attribute__((__packed__))
#else
#error "Define compiler"
#endif
tmp_tuning_message;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
extern size_t TMP_receiveTuningMessage_flag;
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void
TMP_parse_message(
	char *tuningCmd,
	__REGUL_FPT__ *kP,
	__REGUL_FPT__ *kI,
	__REGUL_FPT__ *kD);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif  /* TMP_TUNING_MESSAGE_PARSER_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
