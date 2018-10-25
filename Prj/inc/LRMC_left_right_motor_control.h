/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef LRMC_LEFT_RIGHT_MOTOR_CONTROL_H_
#define LRMC_LEFT_RIGHT_MOTOR_CONTROL_H_


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
#include "stdint.h"
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
#include <xc.h>
#include <uart.h>
#include <dma.h>
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "../../Lib_A_VMCPC_vmc_protocol_communicate/Lib_A_VMCPC_vmc_protocol_communicate.h"
#include "../../Lib_H_PIC_uart/Lib_H_PIC_uart.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
extern vmcpc_f3m_package_s LRMC_leftRightMotorControlPack_s;
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void
LRMC_Init_UART_DMA_IOPins(
	unsigned long fcy,
	unsigned long baudrate);

extern void
LRMC_SendCmdForLeftRightMotors(
	vmcpc_f3m_package_s *p_s,
	__VMCPC_FPT__ leftMotor,
	__VMCPC_FPT__ rightMotor);

extern void
LRMC_StartForce_UART2DMATransmit(
	unsigned int *pMemSrc,
	unsigned int cnt);

extern void
LRMC_Start_UART2DMATramsmit(
	unsigned int *pMemSrc,
	unsigned int cnt);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* LRMC_LEFT_RIGHT_MOTOR_CONTROL_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
