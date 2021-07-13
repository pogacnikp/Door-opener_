/**
  ******************************************************************************
  * @file    motor_control.c
  * @author  Peter Pogacnik
  * @brief   Motor control functions
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"

/* Private function prototypes------------------------------------------------*/
static void MC_setWriteOutFlg(MC_Def_t* motor);
static void MC_calcRaw(MC_Def_t* motor);
static void MC_setDrv(MC_Def_t* motor);

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Initialize max frequency and max raw frequency
  * @param 	frequencyMax  		max frequency value
  * @param 	frequencyMaxRaw  	max raw frequency value
  * @param	motor				motor control structure
  * @retval None
  */
void MC_Freq_InitLimits(MC_Def_t* motor, uint32_t frequencyMax, uint32_t frequencyMaxRaw){
	motor->quantity.Max.Frequency = frequencyMax;
	motor->raw.Max.Frequency = frequencyMaxRaw;
}

/**
  * @brief  Initialize max speed
  * @param 	speedMax  		max speed value
  * @param	motor			motor control structure
  * @retval None
  */
void MC_Speed_InitLimits(MC_Def_t* motor, float speedMax){
	motor->quantity.Max.Speed = speedMax;
}

/**
  * @brief  Set speed and calculate raw speed value
  * @param 	speed  		speed value
  * @param	motor		motor control structure
  * @retval None
  */
void MC_setSpeed(MC_Def_t* motor, float speed){
	if (speed > motor->quantity.Max.Speed){
		motor->quantity.Speed = motor->quantity.Max.Speed;
	} else {
		motor->quantity.Speed = speed;
	}
}

/**
  * @brief  Set frequency and calculate raw frequency value
  * @param 	speed  		frequency value
  * @param	motor		motor control structure
  * @retval None
  */
void MC_setFrequency(MC_Def_t* motor, uint32_t frequency){
	if (frequency > motor->quantity.Max.Frequency){
		motor->quantity.Frequency = motor->quantity.Max.Frequency;
	} else {
		motor->quantity.Frequency = frequency;
	}
}

/**
  * @brief  Set sirection
  * @param 	direction  	motor direction
  * @param	motor		motor control structure
  * @retval None
  */
void MC_setDirection(MC_Def_t* motor, bool direction){
	motor->direction = direction;
}

/**
  * @brief  Start/stop motor
  * @param 	run  		start/stop value
  * @param	motor		motor control structure
  * @retval None
  */
void MC_Run(MC_Def_t* motor, bool run){
	motor->run = run;

	/* Calculate raw frequency and speed */
	MC_calcRaw(motor);

	/* Write raw values in driver accorrding to motor status */
	MC_setDrv(motor);

	/* Set write on output flag */
	MC_setWriteOutFlg(motor);
}

/**
  * @brief  Is motor ON
  * @param	motor		motor control structure
  * @retval TRUE/FALSE
  */
bool MC_isMotorON(MC_Def_t* motor){
	return motor->run;
}

/**
  * @brief  Is flag to write on output setted
  * @param	motor		motor control structure
  * @retval TRUE/FALSE
  */
bool MC_isWriteOutFlgSet(MC_Def_t* motor){
	return motor->writeOut;
}

/**
  * @brief  Clear flag for writing on output
  * @param	motor		motor control structure
  * @retval None
  */
void MC_clearWriteOutFlg(MC_Def_t* motor){
	motor->writeOut = false;
}

/**
  * @brief  Get raw frequency value
  * @param	motor		motor control structure
  * @retval None
  */
uint32_t MC_getRawFrequency(MC_Def_t* motor){
	return motor->raw.Frequency;
}

/**
  * @brief  Get raw speed for driver channel 2
  * @param	motor		motor control structure
  * @retval None
  */
uint32_t MC_getRawSpeedCH1(MC_Def_t* motor){
	return motor->driver.raw1.Speed;
}

/**
  * @brief  Get raw speed for driver channel 2
  * @param	motor		motor control structure
  * @retval None
  */
uint32_t MC_getRawSpeedCH2(MC_Def_t* motor){
	return motor->driver.raw2.Speed;
}

/* Private function ---------------------------------------------------------*/
/**
  * @brief  Set flag to write on output
  * @param	motor		motor control structure
  * @retval None
  */
static void MC_setWriteOutFlg(MC_Def_t* motor){
	motor->writeOut = true;
}

/**
  * @brief  Calculate raw values of frequency and speed
  * @param	motor		motor control structure
  * @retval None
  */
static void MC_calcRaw(MC_Def_t* motor){
	/* Raw frequency is inverted */
	motor->raw.Frequency = motor->raw.Max.Frequency - (motor->quantity.Frequency * (motor->raw.Max.Frequency / motor->quantity.Max.Frequency));

	/* Max raw speed is related to raw frequency */
	motor->raw.Max.Speed = motor->raw.Frequency;
	motor->raw.Speed = motor->quantity.Speed * (motor->raw.Max.Speed / motor->quantity.Max.Speed);
}

/**
  * @brief  Set driver inputs based on motor status
  * @param	motor		motor control structure
  * @retval None
  */
static void MC_setDrv(MC_Def_t* motor){
	if(motor->run == MC_START){
		/* write on driver input based on direction */
		if(motor->direction == MC_FORWARD){
			motor->driver.raw1.Speed = motor->raw.Speed;
			motor->driver.raw2.Speed = 0;
		}
		else if (motor->direction == MC_BACKWARD){
			motor->driver.raw1.Speed = 0;
			motor->driver.raw2.Speed = motor->raw.Speed;
		} else {
			//error
		}
	}
	else if (motor->run == MC_STOP) {
		motor->driver.raw1.Speed = 0;
		motor->driver.raw2.Speed = 0;
	} else {
		//error
	}
}
