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
RBS_Init_IBSCForRetentionDesiredPitchAngle(
	regul_ibsc_s *pIBSC_s);

static void
RBS_Init_PD_ForRetentionDesiredPitchAngle(
	regul_pid_s *pPID_s);

static void
RBS_Init_KI_ForFormationDesiredPitchAngle(
	regul_pid_s *pPID_s);

static void
RBS_Init_D_ForCompensation(
	regul_pid_s *pPID_s);

static void
RBS_GetControlForBalance(
	rbs_balancing_system_s *p_s,
	__PFPT__ pitchAngle,
	__PFPT__ pitchAngularSpeed);

static __PFPT__
RBS_GetDesiredAngle(
	rbs_speed_control_s *pSpeedControl_s,
	__PFPT__ pitchAngle);
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

	RBS_Init_IBSCForRetentionDesiredPitchAngle(
		&p_s->ibscForBalance_s);

	/* Инициализация ПИ регулятора для формирования заданного угла наклона */
	RBS_Init_KI_ForFormationDesiredPitchAngle(
		&p_s->speedControl_s.piRegulator_s);

	RBS_Init_D_ForCompensation(
		&p_s->speedControl_s.dRegulator_s);

	p_s->speedControl_s.compFilt_s.filtCoeff = 0.984;

	p_s->speedControl_s.compFilt_balancedAngle_s.filtCoeff = 0.99999;

	p_s->desiredAngle = (__PFPT__) 0.0;

	p_s->startSystem_flag = 0u;
}

__PFPT__
RBS_GetControlForRobot(
	rbs_balancing_system_s *p_s,
	__PFPT__ pitchAngle,
	__PFPT__ pitchAngularSpeed)
{
	if ((p_s->startSystem_flag == 0u)
			&& (__RBS_fabs((__RBS_fabs(p_s->speedControl_s.balancedAngle) - __RBS_fabs(pitchAngle))) < (__PFPT__) 0.01))
	{
		p_s->startSystem_flag = 1u;
	}
	/* Если угол наклона больше некого значения по модулю, то необходимо
	 * управляющее воздействие установить в нуль */
	if (__RBS_fabs(pitchAngle) > (__PFPT__) 1.5)
	{
		p_s->motorControl						= (__PFPT__) 0.0;
		p_s->motorControl_a[RBS_LEFT_MOTOR]	= (__PFPT__) 0.0;
		p_s->motorControl_a[RBS_RIGHT_MOTOR]	= (__PFPT__) 0.0;
		p_s->speedControl_s.currSpeed			= (__PFPT__) 0.0;
		p_s->speedControl_s.currSpeedFilt		= (__PFPT__) 0.0;
		p_s->speedControl_s.piRegulator_s.integral_s.val = (__PFPT__) 0.0;
		p_s->speedControl_s.piRegulator_s.integral_s.kI = (__PFPT__) 0.0;
		p_s->startSystem_flag = 0u;
	}
	/* Иначе, штатный режим работы */
	else if (p_s->startSystem_flag == 1u)
	{
		/* Текущая скорость задается как управляющее воздействие
		 * на электродвигатели */
		p_s->speedControl_s.currSpeed =
			p_s->motorControl;

		/* Расчет заданного угла наклона */
		p_s->desiredAngle =
			RBS_GetDesiredAngle(
				&p_s->speedControl_s,
				pitchAngle);

		/* Расчет управляющего воздействия для удержания заданного угла наклона */
		RBS_GetControlForBalance(
			p_s,
			pitchAngle,
			pitchAngularSpeed);

		/* Расчет управления для вращения по азимуту */

//        p_s->motorControl_a[RBS_LEFT_MOTOR] = 0.0;
//        p_s->motorControl_a[RBS_RIGHT_MOTOR] = 0.0;

		p_s->motorControl_a[RBS_LEFT_MOTOR] += p_s->speedControl_s.control_data_s.targetRotation;
		p_s->motorControl_a[RBS_RIGHT_MOTOR] -= p_s->speedControl_s.control_data_s.targetRotation;
	}

	return (p_s->motorControl);
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
__PFPT__
RBS_GetDesiredAngle(
	rbs_speed_control_s *pSpeedControl_s,
	__PFPT__ pitchAngle)
{
	/* Фильтрация текущей скорости */
	pSpeedControl_s->currSpeedFilt =
		FILT_Complementary_fpt(
			&pSpeedControl_s->compFilt_s,
			pSpeedControl_s->currSpeed);


	/* Ошибка скорости*/
	__PFPT__ error =
		(pSpeedControl_s->control_data_s.targetSpeed * (1.0 - (__RBS_fabs(pitchAngle) * 1.9)) ) - (pSpeedControl_s->currSpeedFilt);

//	if (pSpeedControl_s->control_data_s.targetSpeed != (__PFPT__) 0.0)
//	{
//	static filt_complementary_s filtTargetSpeed_s;
//	filtTargetSpeed_s.filtCoeff = 0.99;
//	pSpeedControl_s->control_data_s.targetSpeed  =
//		FILT_Complementary_fpt(
//			&filtTargetSpeed_s,
//			pSpeedControl_s->control_data_s.targetSpeed * (1.0 - __RBS_fabs(error)));
//	}
	__PFPT__ desiredAngle = 0.0;

//	static filt_complementary_s filtPidSatur_s;
//	filtPidSatur_s.filtCoeff = 0.99999;
//	pSpeedControl_s->piRegulator_s.pidValSatur =
//		FILT_Complementary_fpt(
//			&filtPidSatur_s,
//			RBS_MAX_ANGLE_RAD  - (__RBS_fabs(error) * ((__PFPT__)1.0)));
//
//	static filt_complementary_s filtPidIntegSatur_s;
//	filtPidIntegSatur_s.filtCoeff = 0.99999;
//	pSpeedControl_s->piRegulator_s.integral_s.satur =
//		FILT_Complementary_fpt(
//			&filtPidIntegSatur_s,
//			RBS_MAX_ANGLE_RAD - (__RBS_fabs(error) * ((__PFPT__)1.0)));
	
	static filt_complementary_s filtError_s;
	filtError_s.filtCoeff = 0.3;
	/* Формирование заданного угла наклона */
	desiredAngle =
		REGUL_Get_PID(
			&pSpeedControl_s->piRegulator_s,
			FILT_Complementary_fpt(
				&filtError_s,
				error),
			NULL);

	if (__RBS_fabs (error) > 0.1)
	{
//		static filt_complementary_s integralCoeffFilt_s;
//		integralCoeffFilt_s.filtCoeff = 0.9997;
//		pSpeedControl_s->piRegulator_s.integral_s.kI =
//			FILT_Complementary_fpt(&integralCoeffFilt_s, (__PFPT__) 4.5);
		pSpeedControl_s->piRegulator_s.integral_s.kI += 0.009;
		if (pSpeedControl_s->piRegulator_s.integral_s.kI > 4.5)
		{
			pSpeedControl_s->piRegulator_s.integral_s.kI = 4.5;
		}

	}
	else
	{
//		static filt_complementary_s integralCoeffFilt_s;
//		integralCoeffFilt_s.filtCoeff = 0.99999;
//		pSpeedControl_s->piRegulator_s.integral_s.kI =
//			FILT_Complementary_fpt(&integralCoeffFilt_s, (__PFPT__) 0.0);
//
//		static filt_complementary_s balanceAngleFilt_s;
//		balanceAngleFilt_s.filtCoeff = 0.1;

//		pSpeedControl_s->piRegulator_s.integral_s.val -=
//			FILT_Complementary_fpt(&balanceAngleFilt_s, (__PFPT__) 0.0);

//		if (pSpeedControl_s->piRegulator_s.integral_s.val > 0.01)
//		{
//			pSpeedControl_s->piRegulator_s.integral_s.val -= 0.0003;
//		}
//		else if(pSpeedControl_s->piRegulator_s.integral_s.val < -0.01)
//		{
//			pSpeedControl_s->piRegulator_s.integral_s.val += 0.0003;
//		}

		pSpeedControl_s->piRegulator_s.integral_s.kI -= 0.008;
		if (pSpeedControl_s->piRegulator_s.integral_s.kI < 0.0)
		{
			pSpeedControl_s->piRegulator_s.integral_s.kI = 0.0;
		}
	}

	if (pSpeedControl_s->control_data_s.targetRotation > 0.0)
	{
		pSpeedControl_s->control_data_s.targetRotation -= __RBS_fabs (error) * 0.19;
	}
	else if (pSpeedControl_s->control_data_s.targetRotation < 0.0)
	{
		pSpeedControl_s->control_data_s.targetRotation += __RBS_fabs (error) * 0.19;
	}
//	if (__RBS_fabs(error) < 0.02)
//	{
//		pSpeedControl_s->balancedAngle =
//			FILT_Complementary_fpt(
//				&pSpeedControl_s->compFilt_balancedAngle_s,
//				pitchAngle);
//
//		/* Корректировака ограничения насыщения ПИ регулятора */
//		pSpeedControl_s->piRegulator_s.pidValSatur =
//			RBS_MAX_ANGLE_RAD + pSpeedControl_s->balancedAngle;
//		pSpeedControl_s->piRegulator_s.integral_s.satur =
//			RBS_MAX_ANGLE_RAD + pSpeedControl_s->balancedAngle;
//	}

//	desiredAngle += REGUL_Get_PID(
//		&pSpeedControl_s->dRegulator_s,
//		-pitchAngle,
//		NULL);

	return (desiredAngle);
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
	p_s->motorControl =
		REGUL_Get_PID(
			&p_s->pdForBalance_s,
			error,
			NULL);

//	p_s->motorControl =
//		REGUL_IBSC(
//			&p_s->ibscForBalance_s,
//			error,
//			p_s->desiredAngle,
//			pitchAngularSpeed);

	/* Копирование найденного значения управляющего воздействия в переменные
	 * для правого и лового моторов */
	p_s->motorControl_a[RBS_LEFT_MOTOR] =
		p_s->motorControl;
	p_s->motorControl_a[RBS_RIGHT_MOTOR] =
		p_s->motorControl;
}


void
RBS_Init_IBSCForRetentionDesiredPitchAngle(
	regul_ibsc_s *pIBSC_s)
{
	regul_ibsc_init_s ibscInit_s;
	REGUL_IBSC_StructInit(&ibscInit_s);
	ibscInit_s.coeff_s.b1		= (__REGUL_FPT__) 2.0;
	ibscInit_s.coeff_s.c1		= (__REGUL_FPT__) 0.05;
	ibscInit_s.coeff_s.c2		= (__REGUL_FPT__) 0.05;
	ibscInit_s.coeff_s.lambda	= (__REGUL_FPT__) 0.0;

	ibscInit_s.dT			= (__REGUL_FPT__) INTEGRATE_PERIOD_IN_SEC;
	ibscInit_s.saturation	= (__REGUL_FPT__) 0.9999;

	REGUL_Init_IBSC(
		pIBSC_s,
		&ibscInit_s);
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
		(__REGUL_FPT__) 75.0;

	/* Коэффициент интегральной составляющей регулятора */
	pidInit_s.kI =
		(__REGUL_FPT__) 0.0;

	/* Коэффициент дифференциальной составляющей регулятора */
	pidInit_s.kD =
		(__REGUL_FPT__) 0.50;

	/* Значение насыщения интегральной составляющей */
	pidInit_s.integralValSaturation =
		(__REGUL_FPT__) 0.9999;

	/* Значение насыщения выходной величины регулятора */
	pidInit_s.returnValSaturation =
		(__REGUL_FPT__) 0.9999;

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
		(__REGUL_FPT__)  0.01;

	/* Коэффициент интегральной составляющей регулятора */
	pidInit_s.kI =
		(__REGUL_FPT__)  1.1;

	/* Значение насыщения интегральной составляющей */
	pidInit_s.integralValSaturation =
		(__REGUL_FPT__) RBS_MAX_ANGLE_RAD;

	/* Значение насыщения выходной величины регулятора */
	pidInit_s.returnValSaturation =
		(__REGUL_FPT__) RBS_MAX_ANGLE_RAD;

	/* Инициализация структуры регулятора */
	REGUL_Init_PID(
		pPID_s,
		&pidInit_s);
}

void
RBS_Init_D_ForCompensation(
	regul_pid_s *pPID_s)
{
	regul_pid_init_struct_s pidInit_s;
	REGUL_PID_StructInit(
		&pidInit_s);

	/* Период интегрирования/дифференцирования */
	pidInit_s.dT =
		(__REGUL_FPT__) INTEGRATE_PERIOD_IN_SEC;

	pidInit_s.kD =
		(__REGUL_FPT__) 0.0001;

	pidInit_s.returnValSaturation =
		(__REGUL_FPT__) 0.01;

	/* Инициализация структуры регулятора */
	REGUL_Init_PID(
		pPID_s,
		&pidInit_s);
}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
