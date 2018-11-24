/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef UDI_UART_DEBUG_INFORMATION_H
#define UDI_UART_DEBUG_INFORMATION_H


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
#include <xc.h>
#include <dma.h>
#include <uart.h>
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "../../Prj/inc/main.h"
#include "../../Lib_H_PIC_uart/Lib_H_PIC_uart.h"
#include "../../Lib_A_DI_debug_information/Lib_A_DI_debug_information.h"
#include "../inc/IISMPU_internal_inert_sens_mpu.h"
#include "../inc/RPA_robot_pitch_angle.h"
#include "../inc/CMP_control_message_parser.h"
#include "../inc/TMP_tuning_message_parser.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
extern DI_data_for_serial_plot_s UDI_serialPlotDataPackage_s __attribute__((eds, space(dma)));
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void
UDI_GetAndSendDebugPackForSerialPlot(
	DI_data_for_serial_plot_s *p_s);

extern void
UDI_Init_All_UART3_RxTx_With_DMA_Tx(
	unsigned int long fcy,
	unsigned int long baudrate);

extern void
UDI_StartForceUart3_DMA3_Transmit(
	unsigned int *pMemSrc,
	unsigned int cnt);

extern void
UDI_StartUart3_DMA3_Transmit(
	unsigned int *pMemSrc,
	unsigned int cnt);


extern void
UDI_StartForceUart3_DMA4_Receiver(
	unsigned int *pMemSrc,
	unsigned int cnt);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* UDI_UART_DEBUG_INFORMATION_H */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
