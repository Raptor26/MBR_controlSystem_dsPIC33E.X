/**
 * @file    CMP_control_message_parser.c
 * @author  Kuroha
 * @version
 * @date    25 октября 2018 г., 12:45
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/CMP_control_message_parser.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*### |Begin| --> Секция - "Глобальные переменные" ##########################*/
size_t CMP_receiveMessage_flag;
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
int16_t
CMP_GetControlVal(
	char* num,
	int16_t saturationVal);
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
void
CMP_init_struct(
	cmp_control_data_s *data)
{
	data->targetRotation  = (__PFPT__) 0.0;
	data->targetSpeed     = (__PFPT__) 0.0;
	data->maxControlValue = (int16_t) 1024;
	data->zeroValue       = (int16_t) (data->maxControlValue / ((__PFPT__) 2.0));
	data->discreteInc     =
		(__PFPT__) ((__PFPT__) 1.0) / ((__PFPT__) data->zeroValue);
}

void
CMP_parse_message(
	cmp_control_data_s *data,
	char *controlCmd)
{
	if (*controlCmd != 0 &&
			controlCmd[5] == '\r' &&
			controlCmd[6] == '\n')
	{
		int16_t number;
		switch (controlCmd[0])
		{
		case 'S':
			// Скорость
			// S0344\r\n
			// sscanf(controlCmd, "S%d\r\n", &number);
			number = CMP_GetControlVal(
						 (char*) &controlCmd[1],
						 data->maxControlValue);
			data->targetSpeed =
				((__PFPT__) (number - data->zeroValue)) * data->discreteInc;
			break;
		case 'D':
			// Направление
			// sscanf(controlCmd, "D%d\r\n", &number);
//			number = (int16_t) atoi((char*) &controlCmd[1]);
//			number = CMP_CheckMinMaxVal(number, data->maxControlValue);
			number = CMP_GetControlVal(
						 (char*) &controlCmd[1],
						 data->maxControlValue);
			data->targetRotation =
				((__PFPT__) (number - data->zeroValue)) * data->discreteInc;
			break;
		}
	}
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
int16_t
CMP_GetControlVal(
	char* num,
	int16_t saturationVal)
{
	int16_t val = (int16_t) atoi(num);
	if (val < 0 || val > saturationVal)
	{
		return saturationVal / 2;
	}
	return val;
}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
