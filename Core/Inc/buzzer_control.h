
/******************************************************************************
  * @file    buzzer_control.h
  * @author  Peter Pogacnik
  * @brief   Buzzer control module header file.
  ****************************************************************************/


#ifndef INC_BUZZER_CONTROL_H_
#define INC_BUZZER_CONTROL_H_

#ifdef __cplusplus

 extern "C" {
#endif /* __cplusplus */

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

 /* Defines -------------------------------------------------------------------*/
/* Buzzer Start/Stop */
#define BZ_OFF			0
#define BZ_ON			1

 /* TypeDefs ------------------------------------------------------------------*/
 /**
   * @brief  Max raw values definition
  */

 typedef struct BZ_RawValueMax {
    uint32_t 	Frequency;		/* Max raw value frequency */
    uint32_t 	DutyCycle;		/* Max raw value duty cycle */
 } BZ_RawMax_t;

 /**
   * @brief  Raw values (values writen in PWM and then on driver) definition
  */

 typedef struct BZ_Raw {
	uint32_t 		Frequency;		/* Frequency raw value */
	uint32_t 		DutyCycle;		/* Duty cycle raw value */
	BZ_RawMax_t 	Max;
 } BZ_Raw_t;

 /**
   * @brief  Max quantity values definition
  */

 typedef struct BZ_QuantityMax {
    uint32_t 		Frequency;			/* Max frequency value */
 } BZ_QuantityMax_t;

 /**
   * @brief  Quantity values definition
  */

 typedef struct BZ_Quantity {
	uint32_t 			Frequency;		/* Frequency value */
	uint32_t	 		DutyCycle;		/* duty cycle value */
	BZ_QuantityMax_t 	Max;
 } BZ_Quantity_t;

 /**
   * @brief  Milliseconds counter definition
  */

 typedef struct BZ_Counter {
	uint32_t 			ms;				/* Counter for beep length */
	bool	 			Enable;			/* Counter enabled for counting */
 } BZ_Counter_t;

 /**
   * @brief  Buzzer driver definition
  */

 typedef struct BZ_Driver {
	uint32_t 			Frequency;		/* Frequency value */
	uint32_t 			DutyCycle;		/* Counter for beep length */
 } BZ_Driver_t;

 /**
   * @brief  Buzzer general definition
  */

 typedef struct BZ_Def {
	bool				set;			/* Set Buzzer OFF/ON */
	bool				writeOut;		/* Flag to write raw on output */
	uint32_t			BeepLength;		/* Beep length in milliseconds */
	BZ_Raw_t	 		raw;
	BZ_Quantity_t  		quantity;
	BZ_Counter_t		count;
	BZ_Driver_t 		driver;			/* Output that goes on PWM and then on buzzer driver */
 } BZ_Def_t;

 /* Function prototypes------------------------------------------------*/
 void BZ_Freq_InitLimits(BZ_Def_t* buzzer, uint32_t frequencyMax, uint32_t frequencyMaxRaw);
 void BZ_setFrequency(BZ_Def_t* buzzer, uint32_t frequency);
 uint32_t BZ_getRawFrequency(BZ_Def_t* buzzer);
 void BZ_setDutyCycle(BZ_Def_t* buzzer, uint32_t dutyCycle);
 uint32_t BZ_getRawDutyCycle(BZ_Def_t* buzzer);
 void BZ_setBeepLength(BZ_Def_t* buzzer, uint32_t beepLength);
 void BZ_setOnOff(BZ_Def_t* buzzer, bool set);
 bool BZ_isBuzzerON(BZ_Def_t* buzzer);
 void BZ_countMillis(BZ_Def_t* buzzer);
 bool BZ_isWriteOutFlgSet(BZ_Def_t* buzzer);
 void BZ_clearWriteOutFlg(BZ_Def_t* buzzer);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_BUZZER_CONTROL_H_ */
