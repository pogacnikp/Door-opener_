
/******************************************************************************
  * @file    motor_control.h
  * @author  Peter Pogacnik
  * @brief   Motor control module header file.
  ****************************************************************************/


#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#ifdef __cplusplus

 extern "C" {
#endif /* __cplusplus */

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

 /* Defines -------------------------------------------------------------------*/
 /* Motor direction */
#define MC_FORWARD  	0
#define MC_BACKWARD 	1

/* Motor Start/Stop */
#define MC_STOP			0
#define MC_START		1

 /* TypeDefs ------------------------------------------------------------------*/
 /**
   * @brief  Max raw values definition
  */

 typedef struct MC_RawValueMax {
    uint32_t 	Frequency;		/* Max raw value frequency */
    uint32_t 	Speed;			/* Max raw value speed */
 } MC_RawMax_t;

 /**
   * @brief  Raw values (values writen in PWM and then on driver) definition
  */

 typedef struct MC_Raw {
	uint32_t 		Frequency;	/* Frequency raw value */
	uint32_t 		Speed;		/* Speed raw value */
	MC_RawMax_t 	Max;
 } MC_Raw_t;

 /**
   * @brief  Max quantity values definition
  */

 typedef struct MC_QuantityMax {
    uint32_t 	Frequency;		/* Max frequency value */
    float	 	Speed;			/* Max speed value */
 } MC_QuantityMax_t;

 /**
   * @brief  Quantity values definition
  */

 typedef struct MC_Quantity {
	uint32_t 			Frequency;	/* Frequency value */
	float	 			Speed;		/* Speed value */
	MC_QuantityMax_t 	Max;
 } MC_Quantity_t;

 /**
   * @brief  Motor driver definition
  */

 typedef struct MC_Driver {
	MC_Raw_t raw1;	/* Channel 1 on driver */
	MC_Raw_t raw2;	/* Channel 2 on driver */
 } MC_Driver_t;

 /**
   * @brief  Motor general definition
  */

 typedef struct MC_Def {
	bool			run;			/* Start/stop motor */
	bool		 	direction;		/* Motor direction */
	bool			writeOut;		/* Flag to write raw on output */
	MC_Raw_t	 	raw;
	MC_Quantity_t  	quantity;
	MC_Driver_t 	driver;
 } MC_Def_t;

 /* Function prototypes------------------------------------------------*/
 void MC_Freq_InitLimits(MC_Def_t* motor, uint32_t frequencyMax, uint32_t frequencyMaxRaw);
 void MC_Speed_InitLimits(MC_Def_t* motor, float speedMax);
 void MC_setSpeed(MC_Def_t* motor, float speed);
 void MC_setFrequency(MC_Def_t* motor, uint32_t frequency);
 void MC_setDirection(MC_Def_t* motor, bool direction);
 void MC_Run(MC_Def_t* motor, bool run);
 bool MC_isMotorON(MC_Def_t* motor);
 bool MC_isWriteOutFlgSet(MC_Def_t* motor);
 void MC_clearWriteOutFlg(MC_Def_t* motor);
 uint32_t MC_getRawFrequency(MC_Def_t* motor);
 uint32_t MC_getRawSpeedCH1(MC_Def_t* motor);
 uint32_t MC_getRawSpeedCH2(MC_Def_t* motor);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_MOTOR_CONTROL_H_ */
