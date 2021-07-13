
/******************************************************************************
  * @file    motor_diag.h
  * @author  Peter Pogacnik
  * @brief   Motor diagnostics module header.
  ****************************************************************************/

#ifndef INC_MOTOR_DIAG_H_
#define INC_MOTOR_DIAG_H_

#ifdef __cplusplus

 extern "C" {
#endif /* __cplusplus */

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

 /* Defines -------------------------------------------------------------------*/

 /* TypeDefs ------------------------------------------------------------------*/
 /**
   * @brief  Circuit status structure definition
   */
 typedef enum
 {
	 MD_UNDEFINED,
	 MD_OPEN_CIRCUT,
	 MD_CLOSED_CIRCUT,
 } MD_CircuitStatus_t;

 /**
   * @brief  Voltage measurement definition
  */

 typedef struct MD_Voltage {
	float			Vplus;				/* Plus motor voltage */
	float			Vminus;				/* Minus motor voltage */
	float			V_per_raw;			/* Current unit per raw unit */
 } MD_Voltage_t;

 /**
   * @brief  Current measurement definition
  */

 typedef struct MD_Current {
	float			I;					/* Motor current */
	float			I_stall;			/* Motor stall current */
	float			I_per_raw;			/* Current unit per raw unit */
 } MD_Current_t;

 /**
   * @brief  Motor diagnostics general definition
  */

 typedef struct MD_Def {
	MD_Current_t 	current;
	MD_Voltage_t  	voltage;
 } MD_Def_t;

 /* Function prototypes------------------------------------------------*/
 void MD_Init_currentStall(MD_Def_t* motor_diag, float currentStall);
 void MD_calibrateCurrent(MD_Def_t* motor_diag, uint32_t valueRaw, float valueCurrent);
 void MD_calibrateVoltage(MD_Def_t* motor_diag, uint32_t valueRaw, float valueVoltage);
 void MD_measureVoltage(MD_Def_t* motor_diag, uint32_t raw_Vplus, uint32_t raw_Vminus);
 void MD_measureCurrent(MD_Def_t* motor_diag, uint32_t raw_I);
 MD_CircuitStatus_t MD_getCircuitStatus(MD_Def_t* motor_diag);
 bool MD_isStall(MD_Def_t* motor_diag);
 bool MD_isEndPosition(MD_Def_t* motor_diag);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_MOTOR_DIAG_H_ */
