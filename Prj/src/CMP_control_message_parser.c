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
	data->targetMaxVal     = 0.7;
	data->maxControlValue = (int16_t) 1024;
	data->maxTimeout      = (uint32_t) 200000u;
	data->zeroValue       = (int16_t) (data->maxControlValue / ((__PFPT__) 2.0));
	data->discreteInc     =
		(__PFPT__) ((__PFPT__) 0.4) / ((__PFPT__) data->zeroValue);

	data->filtForTargetSpeed_s.filtCoeff =
		(__FILT_FPT__) 0.7;
	data->filtForTargetRotation_s.filtCoeff =
		(__FILT_FPT__) 0.05;

	VTMR_InitTimerStruct(
		&data->virtTmr,
		(uint16_t*) &HC32_UPPER_CNT,
		(uint16_t*) &HC32_LOWER_CNT);

	VTMR_StartTimer(&data->virtTmr);
}

cmp_message_status_e
CMP_parse_message(
	cmp_control_data_s *data,
	char *controlCmd)
{
	/* В чем смысл первого условия "if" ? */
	if (*controlCmd != 0
			&& controlCmd[0] == 'S'
			&& controlCmd[5] == '\r'
			&& controlCmd[6] == '\n'
			&& controlCmd[7] == 'D'
			&& controlCmd[12] == '\r'
			&& controlCmd[13] == '\n')
	{
		// Скорость
		int16_t number =
			CMP_GetControlVal(
				(char*) &controlCmd[1],
				data->maxControlValue);
		data->targetSpeed =
			FILT_Complementary_fpt(
				&data->filtForTargetSpeed_s,
				((__PFPT__) (number - data->zeroValue)) * data->discreteInc);
		number =
			CMP_GetControlVal(
				(char*) &controlCmd[8],
				data->maxControlValue);
		data->targetRotation =
			FILT_Complementary_fpt(
				&data->filtForTargetRotation_s,
				((__PFPT__) (number - data->zeroValue)) * data->discreteInc);
		return CMP_MESSAGE_VALID;
	}
	return CMP_MESSAGE_INVALID;
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
int16_t
CMP_GetControlVal(
	char* num,
	int16_t saturationVal)
{
	int16_t val = (int16_t) atoi(num);

	if (val < 0)
	{
		val = 0;
	}
	else if (val > saturationVal)
	{
		val = saturationVal;
	}
//	if (val < 0 || val > saturationVal)
//	{
//        /* FIXME 512 заменить на переменную */
//		return 512;
//	}
	return val;
}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
