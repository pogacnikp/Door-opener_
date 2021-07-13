
/******************************************************************************
  * @file    switch.h
  * @author  Peter Pogacnik
  * @brief   Switch module header file.
  ****************************************************************************/


#ifndef INC_SWITCH_H_
#define INC_SWITCH_H_

#ifdef __cplusplus

 extern "C" {
#endif /* __cplusplus */

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

 /* Defines -------------------------------------------------------------------*/

 /* TypeDefs ------------------------------------------------------------------*/
 /**
   * @brief  Switch logic states structure definition
   */
 typedef enum
 {
	 ST_STATE_LOW,
	 ST_STATE_HIGH,
 } ST_LogicStates_t;

 /**
   * @brief  Switch rising/falling structure definition
   */
 typedef enum
 {
	 ST_EDGE_RISING,
	 ST_EDGE_FALLING,
 } ST_EdgeStatus_t;

 /**
   * @brief  Raw values (values related to configuration)
  */

 typedef struct ST_State {
	ST_LogicStates_t 		current;			/* Current switch state */
	ST_LogicStates_t 		previous;			/* Previous switch state */
 } ST_State_t;

 /**
   * @brief  Milliseconds counter definition
  */

 typedef struct ST_Counter {
	uint32_t 			ms;				/* Counter for debounce in ms */
	bool	 			Enable;			/* Counter enabled for counting */
 } ST_Counter_t;

 /**
   * @brief  Switch general definition
  */

 typedef struct ST_Def {
	uint32_t	 		debTime;		/* Debounce time in ms */
	uint32_t			Level;			/* Switch level */
	ST_EdgeStatus_t		Edge;			/* Edge type - rising/falling */
	ST_Counter_t		count;
	ST_State_t			State;
 } ST_Def_t;

 /* Function prototypes------------------------------------------------*/
 void ST_Init_debounceTime(ST_Def_t* switch_struct, uint32_t debounceTime);
 void ST_Init_SwitchLevel(ST_Def_t* switch_struct, uint32_t switchLevel);
 void ST_enableCounter(ST_Def_t* switch_struct);
 bool ST_isCounted(ST_Def_t* switch_struct);
 void ST_disableCounter(ST_Def_t* switch_struct);
 void ST_countMillis(ST_Def_t* switch_struct);
 void ST_readSwitchState(ST_Def_t* switch_struct, bool switchValue);
 uint32_t ST_getSwitchLevel( ST_Def_t* switch_struct);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_SWITCH_H_ */
