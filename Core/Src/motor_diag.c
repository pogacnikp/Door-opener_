/**
  ******************************************************************************
  * @file    motor_diag.c
  * @author  Peter Pogacnik
  * @brief   Motor diagnostics functions
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "motor_diag.h"

/* Private define ------------------------------------------------------------*/
/* Open circuit threshold for voltage and current */
#define MD_TRHD_OPEN_CIRCUIT	0.01

/* Private function prototypes------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize max amplitude for detection
  * @param 	currentStall	max amplitude value
  * @param	motor_diag		motor diagnostics structure
  * @retval None
  */
void MD_Init_currentStall(MD_Def_t* motor_diag, float currentStall){
	motor_diag->current.I_stall = currentStall;
}

/**
  * @brief  Calibrate conversion between ADC value and current value
  * @param 	valueRaw			ADC sampled value
  * @param	valueCurrent		current value for ADC sampled value
  * @param	motor_diag			motor diagnostics structure
  * @retval None
  */
void MD_calibrateCurrent(MD_Def_t* motor_diag, uint32_t valueRaw, float valueCurrent){
	motor_diag->current.I_per_raw = valueCurrent / valueRaw;
}

/**
  * @brief  Calibrate conversion between ADC value and voltage value
  * @param 	valueRaw			ADC sampled value
  * @param	valueVoltage		voltage value for ADC sampled value
  * @param	motor_diag			motor diagnostics structure
  * @retval None
  */
void MD_calibrateVoltage(MD_Def_t* motor_diag, uint32_t valueRaw, float valueVoltage){
	motor_diag->voltage.V_per_raw = valueVoltage / valueRaw;
}

/**
  * @brief  Measure raw voltage and convert to physical value
  * @param 	raw_Vplus			Vplus raw voltage from ADC
  * @param	raw_Vminus			Vminus raw voltage from ADC
  * @param	motor_diag			motor diagnostics structure
  * @retval None
  */
void MD_measureVoltage(MD_Def_t* motor_diag, uint32_t raw_Vplus, uint32_t raw_Vminus){

	if (motor_diag->voltage.V_per_raw == 0){
		motor_diag->voltage.Vplus = raw_Vplus / motor_diag->voltage.V_per_raw;
		motor_diag->voltage.Vminus = raw_Vminus / motor_diag->voltage.V_per_raw;
	} else {
		//Calibration error
	}
}

/**
  * @brief  Measure raw voltage and convert to physical value
  * @param 	raw_I				raw current from ADC
  * @param	motor_diag			motor diagnostics structure
  * @retval None
  */
void MD_measureCurrent(MD_Def_t* motor_diag, uint32_t raw_I){

	if (motor_diag->current.I_per_raw == 0){
		motor_diag->current.I = raw_I / motor_diag->current.I_per_raw;
	} else {
		//Calibration error
	}
}

/**
  * @brief  Return the circuit status
  * @param	motor_diag			motor diagnostics structure
  * @retval CircuitStatus
  */
MD_CircuitStatus_t MD_getCircuitStatus(MD_Def_t* motor_diag){

	/* Voltage and current are present (above threshold) */
	if (motor_diag->current.I > MD_TRHD_OPEN_CIRCUIT &&
	    (motor_diag->voltage.Vplus > MD_TRHD_OPEN_CIRCUIT || motor_diag->voltage.Vminus > MD_TRHD_OPEN_CIRCUIT)){
		return MD_CLOSED_CIRCUT;
	}
	/* Voltage is present, current is not */
	else if (motor_diag->current.I < MD_TRHD_OPEN_CIRCUIT &&
		     (motor_diag->voltage.Vplus > MD_TRHD_OPEN_CIRCUIT || motor_diag->voltage.Vminus > MD_TRHD_OPEN_CIRCUIT)){
		return MD_OPEN_CIRCUT;

	} else {
		return MD_UNDEFINED;
	}
}

/**
  * @brief  Return true if motor is in stall, else return false
  * @param	motor_diag			motor diagnostics structure
  * @retval true/false
  */
bool MD_isStall(MD_Def_t* motor_diag){

	/* Current is higher or equal to defined stall current */
	if (motor_diag->current.I >= motor_diag->current.I_stall){
		return true;
	} else {
		return false;
	}
}

/**
  * @brief  Return true if motor is at the end position, else return false
  * @param	motor_diag			motor diagnostics structure
  * @retval true/false
  */
bool MD_isEndPosition(MD_Def_t* motor_diag){

	/* When motor is in stall, end position is reached*/
	return MD_isStall(motor_diag);
}

/* Private functions ---------------------------------------------------------*/
