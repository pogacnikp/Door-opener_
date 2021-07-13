
/******************************************************************************
  * @file    power_supply_det.h
  * @author  Peter Pogacnik
  * @brief   Power supply detection module header.
  ****************************************************************************/

#ifndef INC_POWER_SUPPLY_DET_H_
#define INC_POWER_SUPPLY_DET_H_

#ifdef __cplusplus

 extern "C" {
#endif /* __cplusplus */

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

 /* Defines -------------------------------------------------------------------*/

 /* TypeDefs ------------------------------------------------------------------*/

 /**
   * @brief  DC power supply rising/falling structure definition
   */
 typedef enum
 {
	 PS_EDGE_UNDEFINED,
	 PS_EDGE_RISING,
	 PS_EDGE_FALLING,
 } PS_EdgeStatus_t;

 /**
    * @brief  Power supply type structure definition
    */
 typedef enum
  {
	 PS_UNDEFINED,
	 PS_AC,
	 PS_DC,
  } PS_PowerSupplyType_t;

 /**
   * @brief  DC power supply definition
  */

 typedef struct PS_DC_Def {
	PS_EdgeStatus_t		Edge;				/* Edge type - rising/falling */
 } PS_DC_Def_t;

 /**
   * @brief  AC power supply definition
  */

 typedef struct PS_AC_Def {
	uint32_t		T;					/* Period (ms) to detect in AC mode*/
	bool			isT;				/* Current period (ms) period is same as T */
 } PS_AC_Def_t;

 /**
   * @brief  Samples counting based on sample value
  */

 typedef struct PS_SampleCounter {
	uint32_t		sampleNum;			/* Current number of samples in period */
	uint32_t		High;				/* High samples in period */
	uint32_t		Low;				/* Low samples in period */
 } PS_SampleCounter_t;

 /**
   * @brief  End counting values
  */

 typedef struct PS_CountEnd {
 	uint32_t		DC;				/* Number to end counting (signal is DC) */
 	uint32_t		AC_T;			/* Period end counting when signal is AC with period T */
  } PS_CountEnd_t;

 /**
   * @brief  Sample structure
  */

 typedef struct PS_Sample {
	float 					currSampleVal;						/* Current sample value */
	uint32_t				currSampleLevel;					/* Current sample level (init as undefined) */
	uint32_t				prevSampleLevel;					/* Previous sample level (init as undefined) */
	PS_SampleCounter_t		currCounter;						/* Current period counter structure */
	PS_SampleCounter_t		prevCounter;						/* Previous period counter structure */
	PS_CountEnd_t			countEnd;							/* End counting structure */
 } PS_Sample_t;

 /**
   * @brief  Power supply detection general definition
  */

 typedef struct PS_Def {
	PS_PowerSupplyType_t	supplyType;			/* Supply type AC/DC */
	uint32_t	 			Ts;					/* Sampling period (μs) */
	float	 				maxAmplitude;		/* Max amplitude value */
	PS_DC_Def_t  			DC;
	PS_AC_Def_t 			AC;
	PS_Sample_t				sample;
 } PS_Def_t;

 /* Function prototypes------------------------------------------------*/
 void PS_Init_maxAmplitude(PS_Def_t* power_supply, float maxAmplitude);
 void PS_Init_samplingPeriod(PS_Def_t* power_supply, uint32_t samplingPeriod);
 void PS_Init_detectionPeriod(PS_Def_t* power_supply, uint32_t detectionPeriod);
 void PS_readSample(PS_Def_t* power_supply, float sample);
 PS_PowerSupplyType_t PS_getSupplyType(PS_Def_t* power_supply);
 bool PS_isTcorrect(PS_Def_t* power_supply);
 PS_EdgeStatus_t PS_getDCEdgeType(PS_Def_t* power_supply);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_POWER_SUPPLY_DET_H_ */
