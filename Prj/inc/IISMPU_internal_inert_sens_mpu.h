/**
 * @file   	IISMPU_internal_inert_sens_mpu.
 * @author 	Kuroha
 * @version
 * @date 	28 сентября 2018 г., 10:54
 * @brief
 */


#ifndef IISMPU_INTERNAL_INERT_SENS_MPU_H_
#define IISMPU_INTERNAL_INERT_SENS_MPU_H_


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
#include <xc.h>
#include <libpic30.h>
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "../../Lib_H_MPU60x0_inertial_sensor/Lib_H_mpu60x0_inertial_sensor.h"
#include "../../Lib_H_PIC_spi/Lib_H_PIC_spi.h"
#include "../inc/main.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
typedef enum
{
	IISMPU_ROLL = 0,
	IISMPU_PITCH,
	IISMPU_YAW,

	/**
	 * @brief Данное определение должно быть крайним в данном перечисляемом типе.
	 *        Оно используется для определения размеров векторов показаний
	 *        инерциального датчика
	 */
	IISMPU_VECT_SIZE,
} iismpu_coordinate_system_definitions_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
extern mpu60x0_spi_s IISMPU_SPIFnc_s;
extern mpu60x0_lsb_s IISMPU_LSB_s;
extern mpu60x0_data_s IISMPU_data_s;
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void
IISMPU_Init_AllPeriphForInternalMPU6000(
	void);

extern void
IISMPU_GetAccGyrTemperature(
	float *pAcc,
	float *pGyr,
	float *pTemperature);

extern void
IISMPU_SetNEDCoordinateSystem(
	float pData[]);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* IISMPU_INTERNAL_INERT_SENS_MPU_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
