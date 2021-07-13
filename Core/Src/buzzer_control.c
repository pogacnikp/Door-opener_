/**
  ******************************************************************************
  * @file    Buzzer_control.c
  * @author  Peter Pogacnik
  * @brief   Buzzer control functions
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "buzzer_control.h"

/* Private function prototypes------------------------------------------------*/
static void BZ_setWriteOutFlg(BZ_Def_t* buzzer);
static void BZ_calcRaw(BZ_Def_t* buzzer);
static void BZ_setBuzzerDrv(BZ_Def_t* buzzer);
static void BZ_enableCounter(BZ_Def_t* buzzer);
static void BZ_resetCounter(BZ_Def_t* buzzer);

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Initialize max frequency and max raw frequency
  * @param 	frequencyMax  		max frequency value
  * @param 	frequencyMaxRaw  	max raw frequency value
  * @param	buzzer				buzzer control structure
  * @retval None
  */
void BZ_Freq_InitLimits(BZ_Def_t* buzzer, uint32_t frequencyMax, uint32_t frequencyMaxRaw){
	buzzer->quantity.Max.Frequency = frequencyMax;
	buzzer->raw.Max.Frequency = frequencyMaxRaw;
}

/**
  * @brief  Set frequency
  * @param 	frequency  			frequency value
  * @param	buzzer				buzzer control structure
  * @retval None
  */
void BZ_setFrequency(BZ_Def_t* buzzer, uint32_t frequency){
	if (frequency > buzzer->quantity.Max.Frequency){
		buzzer->quantity.Frequency = buzzer->quantity.Max.Frequency;
	} else {
		buzzer->quantity.Frequency = frequency;
	}
}

/**
  * @brief  Get raw frequency value
  * @param	buzzer				buzzer control structure
  * @retval None
  */
uint32_t BZ_getRawFrequency(BZ_Def_t* buzzer){
	return buzzer->raw.Frequency;
}

/**
  * @brief  Set duty cycle and calculate raw duty cycle value
  * @param 	dutyCycle	  		duty cycle value
  * @param	buzzer				buzzer control structure
  * @retval None
  */
void BZ_setDutyCycle(BZ_Def_t* buzzer, uint32_t dutyCycle){

	/* Max duty cycle is 100% */
	if (dutyCycle > 100){
		buzzer->quantity.DutyCycle = 100;
	} else {
		buzzer->quantity.DutyCycle = dutyCycle;
	}
}

/**
  * @brief  get raw duty cycle value
  * @param	buzzer				buzzer control structure
  * @retval None
  */
uint32_t BZ_getRawDutyCycle(BZ_Def_t* buzzer){
	return buzzer->raw.DutyCycle;
}

/**
  * @brief  Set beep length in milliseconds
  * @param 	beepLength  		beep length
  * @param	buzzer				buzzer control structure
  * @retval None
  */
void BZ_setBeepLength(BZ_Def_t* buzzer, uint32_t beepLength){
		buzzer->BeepLength = beepLength;
}

/**
  * @brief  Set buzzer On/Off
  * @param 	set  		On/Off value
  * @param	buzzer		buzzer control structure
  * @retval None
  */
void BZ_setOnOff(BZ_Def_t* buzzer, bool set){
	buzzer->set = set;

	/* Calculate raw frequency and duty cycle */
	BZ_calcRaw(buzzer);

	/* Write raw values in driver accorrding to buzzer status */
	BZ_setBuzzerDrv(buzzer);

	/* If buzzer is set ON, then start beep length counter */
	if (buzzer->set == BZ_ON){
		BZ_enableCounter(buzzer);
	} else {
		BZ_resetCounter(buzzer);
	}

	/* set write on output flag */
	BZ_setWriteOutFlg(buzzer);
}

/**
  * @brief  Is buzzer ON
  * @param	buzzer		buzzer control structure
  * @retval TRUE/FALSE
  */
bool BZ_isBuzzerON(BZ_Def_t* buzzer){
	return buzzer->set;
}

/**
  * @brief  Call in timer ISR with 1ms to count each passed ms
  * @param	buzzer		buzzer control structure
  * @retval None
  */
void BZ_countMillis(BZ_Def_t* buzzer){

	/* Count only if buzzer is enabled */
	if (buzzer->count.Enable == true){
		buzzer->count.ms++;

		/* After beep length reset counter and set buzzer to Off */
		if (buzzer->count.ms > buzzer->BeepLength){
			buzzer->count.ms = 0;

			BZ_setOnOff(BZ_OFF, buzzer);
		}
	}
}

/**
  * @brief  Is flag to write on output setted
  * @param	buzzer		buzzer control structure
  * @retval TRUE/FALSE
  */
bool BZ_isWriteOutFlgSet(BZ_Def_t* buzzer){
	return buzzer->writeOut;
}

/**
  * @brief  Clear flag for writing on output
  * @param	buzzer		buzzer control structure
  * @retval None
  */
void BZ_clearWriteOutFlg(BZ_Def_t* buzzer){
	buzzer->writeOut = false;
}

/* Private function ---------------------------------------------------------*/
/**
  * @brief  Set flag to write on output
  * @param	buzzer		buzzer control structure
  * @retval None
  */
static void BZ_setWriteOutFlg(BZ_Def_t* buzzer){
	buzzer->writeOut = true;
}

/**
  * @brief  Set driver inputs based on buzzer status
  * @param	buzzer		buzzer control structure
  * @retval None
  */
static void BZ_setBuzzerDrv(BZ_Def_t* buzzer){

	/* Write in driver DutyCycle, if buzzer ON, else write 0  */
	if(buzzer->set == BZ_ON){
		buzzer->driver.DutyCycle = buzzer->raw.DutyCycle;
	} else {
		buzzer->driver.DutyCycle = 0;
	}

	buzzer->driver.Frequency = buzzer->raw.Frequency;
}

/**
  * @brief  Calculate raw values of frequency and duty cycle
  * @param	buzzer		buzzer control structure
  * @retval None
  */
static void BZ_calcRaw(BZ_Def_t* buzzer){
	/* Raw frequency is inverted */
	buzzer->raw.Frequency = buzzer->raw.Max.Frequency - (buzzer->quantity.Frequency * (buzzer->raw.Max.Frequency / buzzer->quantity.Max.Frequency));

	/* Max raw duty cycle is related to raw frequency */
	buzzer->raw.Max.DutyCycle = buzzer->raw.Frequency;
	buzzer->raw.DutyCycle = buzzer->quantity.DutyCycle * (buzzer->raw.Max.DutyCycle / 100);
}

/**
  * @brief  Enable beep length counter
  * @param	buzzer		buzzer control structure
  * @retval None
  */
static void BZ_enableCounter(BZ_Def_t* buzzer){
	buzzer->count.Enable = true;
}

/**
  * @brief  Reset beep length counter
  * @param	buzzer		buzzer control structure
  * @retval None
  */
static void BZ_resetCounter(BZ_Def_t* buzzer){
	buzzer->count.Enable = false;
	buzzer->count.ms = 0;
}
