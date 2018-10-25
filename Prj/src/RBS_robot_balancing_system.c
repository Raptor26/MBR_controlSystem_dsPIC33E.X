/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "../inc/RBS_robot_balancing_system.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
rbs_balancing_system_s RBS_balancingSystem_s;
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
static void
RBS_Init_PD_ForRetentionDesiredPitchAngle(
	regul_pid_s *pPID_s);

static void
RBS_Init_KI_ForFormationDesiredPitchAngle(
	regul_pid_s *pPID_s);

static void
RBS_GetControlForBalance(
	rbs_balancing_system_s *p_s,
	__PFPT__ pitchAngle,
	__PFPT__ pitchAngularSpeed);
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
void
RBS_Init_BalancingSystem(
	rbs_balancing_system_s *p_s)
{
	/* Инициализация ПД регулятора для удержания заданного угла наклона */
	RBS_Init_PD_ForRetentionDesiredPitchAngle(
		&p_s->pdForBalance_s);

	/* Инициализация ПИ регулятора для формирования заданного угла наклона */
	RBS_Init_KI_ForFormationDesiredPitchAngle(
		&p_s->piForErr);

	p_s->compFilt_s.filtCoeff = 0.85;
	p_s->desiredAngle = (__PFPT__) 0.0;
}

__PFPT__
RBS_GetMotorControlForBalancingRobot(
	rbs_balancing_system_s *p_s,
	__PFPT__ pitchAngle,
	__PFPT__ pitchAngularSpeed)
{

	/* Если угол наклона больше некого значения по модулю, то необходимо
	 * управляющее воздействие установить в нуль */
	if (fabsf((float)pitchAngle) > (float) 1.0)
	{
		p_s->motorControl_a[RBS_RIGHT_MOTOR] = (__PFPT__) 0.0;
	}
	/* Иначе, штатный режим работы */
	else
	{
		/* Расчет заданного угла наклона */
		
		
		/* Расчет управляющего воздействия для удержания заданного угла наклона */
		RBS_GetControlForBalance(
		p_s,
		pitchAngle,
		pitchAngularSpeed);
	}

	return (p_s->motorControl_a[RBS_RIGHT_MOTOR]);
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
void
RBS_GetDesiredAngle(
	rbs_balancing_system_s *p_s)
{
	
}


void
RBS_GetControlForBalance(
	rbs_balancing_system_s *p_s,
	__PFPT__ pitchAngle,
	__PFPT__ pitchAngularSpeed)
{
	/* Ошибка угла наклона как разница между заданным углом и текущим */
	/* FIXME поменять слагаемые местами в соответствии с текстом выше */
	__PFPT__ error =
		pitchAngle - p_s->desiredAngle;

	/* Получить управляющее воздействие для балансирования */
	p_s->motorControl_a[RBS_RIGHT_MOTOR] =
		REGUL_Get_PID(
			&p_s->pdForBalance_s,
			error,
			NULL);

	/* Управляющее воздействие для правого и левого моторов одно и тоже */
	p_s->motorControl_a[RBS_LEFT_MOTOR] =
		p_s->motorControl_a[RBS_RIGHT_MOTOR];
}


/**
 * @brief	Функция выполняет инициализацию структуры ПД регулятора который
 *        	приводит текущий угол наклона балансирующего робота к заданному
 * @param[in]	*pPID_s:	Указатель на структуру ПИД регулятора
 * @return 	None
 */
void
RBS_Init_PD_ForRetentionDesiredPitchAngle(
	regul_pid_s *pPID_s)
{
	regul_pid_init_struct_s pidInit_s;
	REGUL_PID_StructInit(
		&pidInit_s);

	/* Период интегрирования/дифференцирования */
	pidInit_s.dT =
		(__REGUL_FPT__) INTEGRATE_PERIOD_IN_SEC;

	/* Коэффициент пропорциональной составляющей регулятора */
	pidInit_s.kP =
		(__REGUL_FPT__) 30.0;

	/* Коэффициент интегральной составляющей регулятора */
	pidInit_s.kI =
		(__REGUL_FPT__) 0.0;

	/* Коэффициент дифференциальной составляющей регулятора */
	pidInit_s.kD =
		(__REGUL_FPT__) 0.15;

	/* Значение насыщения интегральной составляющей */
	pidInit_s.integralValSaturation =
		(__REGUL_FPT__) 1.0;

	/* Значение насыщения выходной величины регулятора */
	pidInit_s.returnValSaturation =
		(__REGUL_FPT__) 1.0;

	/* Инициализация структуры регулятора */
	REGUL_Init_PID(
		pPID_s,
		&pidInit_s);
}

/**
 * @brief	Функция выполняет инициализацию структуры ПИ регулятора который
 *        	задает необходимый угол наклона балансирующего робота
 * @param[in]	*pPID_s:	Указатель на структуру ПИД регулятора
 * @return 	None
 */
void
RBS_Init_KI_ForFormationDesiredPitchAngle(
	regul_pid_s *pPID_s)
{
	regul_pid_init_struct_s pidInit_s;
	REGUL_PID_StructInit(
		&pidInit_s);

	/* Период интегрирования/дифференцирования */
	pidInit_s.dT =
		(__REGUL_FPT__) INTEGRATE_PERIOD_IN_SEC;

	/* Коэффициент пропорциональной составляющей регулятора */
	pidInit_s.kP =
		(__REGUL_FPT__) 0.5;

	/* Коэффициент интегральной составляющей регулятора */
	pidInit_s.kI =
		(__REGUL_FPT__) 3.0;

	/* Значение насыщения интегральной составляющей */
	pidInit_s.integralValSaturation =
		(__REGUL_FPT__) 1.0;

	/* Значение насыщения выходной величины регулятора */
	pidInit_s.returnValSaturation =
		(__REGUL_FPT__) 1.0;

	/* Инициализация структуры регулятора */
	REGUL_Init_PID(
		pPID_s,
		&pidInit_s);
}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
