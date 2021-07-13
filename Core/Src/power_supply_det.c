/**
  ******************************************************************************
  * @file    power_supply_det.c
  * @author  Peter Pogacnik
  * @brief   Power supply detection functions
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "power_supply_det.h"

/* Private typedef -----------------------------------------------------------*/
 /**
   * @brief  PS logic levels structure definition
   */
 typedef enum
 {
	 PS_LEVEL_UNDEFINED,
	 PS_LEVEL_LOW,
	 PS_LEVEL_HIGH,
 } PS_LogicLevels_t;

/* Private define ------------------------------------------------------------*/

/* Private function prototypes------------------------------------------------*/
static void PS_initCounters(PS_Def_t* power_supply);
static void PS_updateCounters(PS_Def_t* power_supply);
static PS_LogicLevels_t PS_getSampleLevel(PS_Def_t* power_supply);
static PS_PowerSupplyType_t PS_isDC(PS_Def_t* power_supply);
static PS_PowerSupplyType_t PS_isAC(PS_Def_t* power_supply);
static void PS_curr2prev(PS_Def_t* power_supply);
static void PS_clearCounters(PS_SampleCounter_t* sampleCounter);
static void PS_detectDCEdgeType(PS_Def_t* power_supply);
static void PS_checkACPeriod(PS_Def_t* power_supply);
static void PS_processSample(PS_Def_t* power_supply);

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Initialize structure values
  * @param	power_supply	power supply detection structure
  * @retval None
  */
void PS_Init(PS_Def_t* power_supply){
	/* Set sample level to undefined */
	power_supply->sample.currSampleLevel = PS_LEVEL_UNDEFINED;
	power_supply->sample.prevSampleLevel = PS_LEVEL_UNDEFINED;
}

/**
  * @brief  Initialize max amplitude for detection
  * @param 	maxAmplitude	max amplitude value
  * @param	power_supply	power supply detection structure
  * @retval None
  */
void PS_Init_maxAmplitude(PS_Def_t* power_supply, float maxAmplitude){
	power_supply->maxAmplitude = maxAmplitude;
}

/**
  * @brief  Initialize sampling period
  * @param 	samplingPeriod  	sampling period in μs
  * @param	power_supply		power supply detection structure
  * @retval None
  */
void PS_Init_samplingPeriod(PS_Def_t* power_supply, uint32_t samplingPeriod){
	power_supply->Ts = samplingPeriod;

	/* Init samples counters */
	PS_initCounters(power_supply);
}

/**
  * @brief  Initialize detection period
  * @param 	samplingPeriod  	AC detection period in ms
  * @param	power_supply		power supply detection structure
  * @retval None
  */
void PS_Init_detectionPeriod(PS_Def_t* power_supply, uint32_t detectionPeriod){
	power_supply->AC.T = detectionPeriod;

	/* Init samples counters */
	PS_initCounters(power_supply);
}

/**
  * @brief  Read supply voltage sample
  * @param 	sample  			sample value
  * @param	power_supply		power supply detection structure
  * @retval None
  */
void PS_readSample(PS_Def_t* power_supply, float sample){
	power_supply->sample.currSampleVal = sample;

	/* Processing based on new sample */
	PS_processSample(power_supply);
}

/**
  * @brief  Return supply type
  * @param	power_supply		power supply detection structure
  * @retval UNDEFINED/AC/DC
  */
PS_PowerSupplyType_t PS_getSupplyType(PS_Def_t* power_supply){
	return power_supply->supplyType;
}

/**
  * @brief  Return true/false if signal period is as defined detection period
  * @param	power_supply		power supply detection structure
  * @retval TRUE/FALSE
  */
bool PS_isTcorrect(PS_Def_t* power_supply){
	return power_supply->AC.isT;
}

/**
  * @brief  Return DC edge type
  * @param	power_supply		power supply detection structure
  * @retval UNDEFINED/RISING/FALLING
  */
PS_EdgeStatus_t PS_getDCEdgeType(PS_Def_t* power_supply){
	return power_supply->DC.Edge;
}

/* Private function ---------------------------------------------------------*/
/**
  * @brief  Clear counters
  * @param	power_supply		power supply detection structure
  * @retval None
  */
static void PS_clearCounters(PS_SampleCounter_t* sampleCounter){
	sampleCounter->sampleNum = 0;
	sampleCounter->High = 0;
	sampleCounter->Low = 0;
}

/**
  * @brief  Init samples counting
  * @param	power_supply		power supply detection structure
  * @retval None
  */
static void PS_initCounters(PS_Def_t* power_supply){
	PS_clearCounters(&power_supply->sample.currCounter);

	PS_clearCounters(&power_supply->sample.prevCounter);

	/* Ts must be initialized */
	if (power_supply->Ts > 0){
		/* Calculate number of counted samples for DC signal detection.
		 * If state of signal doesn't change for 1s, then signal is supposed as DC.
		 * */
		power_supply->sample.countEnd.DC = 1000000 / power_supply->Ts;
	}

	/* Ts and T must be initialized */
	if (power_supply->Ts > 0 && power_supply->AC.T > 0){
		/* Calculate number of counted samples for period  T */
		power_supply->sample.countEnd.AC_T = (power_supply->AC.T * 1000) / power_supply->Ts;
	}

}

/**
  * @brief  Update counter values
  * @param	power_supply		power supply detection structure
  * @retval None
  */
static void PS_updateCounters(PS_Def_t* power_supply){

	if (power_supply->sample.currSampleLevel == PS_LEVEL_HIGH){
		/* High level counter + 1 */
		power_supply->sample.currCounter.High++;
	}
	else if (power_supply->sample.currSampleLevel == PS_LEVEL_LOW){
		/* Low level counter + 1 */
		power_supply->sample.currCounter.Low++;
	} else {
		//error
	}
}

/**
  * @brief  Get sample state (High/Low)
  * @param	power_supply		power supply detection structure
  * @retval UNDEFINED/LOW/HIGH
  */
static PS_LogicLevels_t PS_getSampleLevel(PS_Def_t* power_supply){

	/* Sample value is LOW if it is bellow 10% of maximum amplitude */
	if (power_supply->sample.currSampleVal < 0.1 * power_supply->maxAmplitude){
		return PS_LEVEL_LOW;
	}
	/* Sample value is HIGH if it is under 90% of maximum amplitude */
	else if (power_supply->sample.currSampleVal > 0.9 * power_supply->maxAmplitude){
		return PS_LEVEL_HIGH;
	} else {
		//error
		return PS_LEVEL_UNDEFINED;
	}

}

/**
  * @brief  Check if power supply is DC
  * @param	power_supply		power supply detection structure
  * @retval UNDEFINED/PS_DC
  */
static PS_PowerSupplyType_t PS_isDC(PS_Def_t* power_supply){

	/* If sample number in period is higher then end counting value, supply is DC */
	if (power_supply->sample.currCounter.sampleNum >= power_supply->sample.countEnd.DC){
		return PS_DC;
	} else {
		return PS_UNDEFINED;
	}

}

/**
  * @brief  Check if power supply is AC
  * @param	power_supply		power supply detection structure
  * @retval UNDEFINED/AC
  */
static PS_PowerSupplyType_t PS_isAC(PS_Def_t* power_supply){
	uint32_t prevCounterNumber = power_supply->sample.prevCounter.sampleNum;
	uint32_t currCounterNumber = power_supply->sample.currCounter.sampleNum;

	/* If sample number in period is the same as previous (0.1% tolerance), then supply is AC */
	if ((currCounterNumber <= prevCounterNumber + 0.001 * prevCounterNumber) && (currCounterNumber >= prevCounterNumber - 0.001 * prevCounterNumber)){
		return PS_AC;
	} else {
		return PS_UNDEFINED;
	}

}

/**
  * @brief  Write current to previous variables
  * @param	power_supply		power supply detection structure
  * @retval None
  */
static void PS_curr2prev(PS_Def_t* power_supply){
	power_supply->sample.prevCounter.sampleNum = power_supply->sample.currCounter.sampleNum;
	power_supply->sample.prevCounter.High = power_supply->sample.currCounter.High;
	power_supply->sample.prevCounter.Low = power_supply->sample.currCounter.Low;

	power_supply->sample.prevSampleLevel = power_supply->sample.currSampleLevel;
}

/**
  * @brief  Detect DC edge type (rissing/falling)
  * @param	power_supply		power supply detection structure
  * @retval None
  */
static void PS_detectDCEdgeType(PS_Def_t* power_supply){

	/* If LOW to HIGH then RISING, if HIGH to LOW, then FALLING */
	if (power_supply->sample.prevSampleLevel == PS_LEVEL_LOW && power_supply->sample.currSampleLevel == PS_LEVEL_HIGH){
		power_supply->DC.Edge = PS_EDGE_RISING;
	}
	else if (power_supply->sample.prevSampleLevel == PS_LEVEL_HIGH && power_supply->sample.currSampleLevel == PS_LEVEL_LOW) {
		power_supply->DC.Edge = PS_EDGE_FALLING;
	}
}

/**
  * @brief  Check if period is as defined
  * @param	power_supply		power supply detection structure
  * @retval None
  */
static void PS_checkACPeriod(PS_Def_t* power_supply){

	uint32_t currSampleNum = power_supply->sample.currCounter.sampleNum;
	uint32_t countEndSampleNum = power_supply->sample.countEnd.AC_T;

	/* If period is the same (0.1% tolerance) as detection period */
	if ((currSampleNum <= countEndSampleNum + 0.001 * countEndSampleNum) && (currSampleNum >= countEndSampleNum - 0.001 * countEndSampleNum)){
		power_supply->AC.isT = true;
	} else {
		power_supply->AC.isT = false;
	}
}

/**
  * @brief  Process the sample value and related internal variables
  * @param	power_supply		power supply detection structure
  * @retval None
  */
static void PS_processSample(PS_Def_t* power_supply){

	power_supply->sample.currSampleLevel = PS_getSampleLevel(power_supply);

	PS_updateCounters(power_supply);

	/* Check power supply type */
	if (PS_isDC(power_supply) == PS_DC){
		power_supply->supplyType = PS_DC;

		/* Detect if signal is Rising/Falling */
		PS_detectDCEdgeType(power_supply);

		PS_curr2prev(power_supply);
		PS_clearCounters(&power_supply->sample.currCounter);
	}
	else if (PS_isAC(power_supply) == PS_AC){
		power_supply->supplyType = PS_AC;

		/* Check if period is T */
		PS_checkACPeriod(power_supply);

		PS_curr2prev(power_supply);
		PS_clearCounters(&power_supply->sample.currCounter);
	}
}
