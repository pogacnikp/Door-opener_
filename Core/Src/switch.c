/**
  ******************************************************************************
  * @file    switch.c
  * @author  Peter Pogacnik
  * @brief   Switch functions
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "switch.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Max switch levels (not logic levels) */
#define ST_SWITCH_LEVEL_MAX			4

/* Private function prototypes------------------------------------------------*/
static void ST_detectEdgeType(ST_Def_t* switch_struct);
static void ST_updateSwitchLevel(ST_Def_t* switch_struct);
static void ST_processState(ST_Def_t* switch_struct);

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Init debounce time in milliseconds
  * @param 	debounceTime  		switch debouncing time in ms
  * @param	switch_struct		switch structure
  * @retval None
  */
void ST_Init_debounceTime(ST_Def_t* switch_struct, uint32_t debounceTime){
	switch_struct->debTime = debounceTime;
}

/**
  * @brief  Init switch level
  * @param 	switchLevel  		switch level
  * @param	switch_struct		switch structure
  * @retval None
  */
void ST_Init_SwitchLevel(ST_Def_t* switch_struct, uint32_t switchLevel){

	/* If initialized level is out of limits, set level to 1 */
	if (switchLevel > ST_SWITCH_LEVEL_MAX || switchLevel == 0){
		switch_struct->Level = 1;
	} else {
		switch_struct->Level = switchLevel;
	}
}

/**
  * @brief  Enable counter to start counting
  * @param	switch_struct		switch structure
  * @retval None
  */
void ST_enableCounter(ST_Def_t* switch_struct){
	switch_struct->count.Enable = true;
}

/**
  * @brief  Return true if counter reached desired value, else false
  * @param	switch_struct		switch structure
  * @retval TRUE/FALSE
  */
bool ST_isCounted(ST_Def_t* switch_struct){
	return switch_struct->count.ms >= switch_struct->debTime;
}

/**
  * @brief  Disable and reset counter
  * @param	switch_struct		switch structure
  * @retval None
  */
void ST_disableCounter(ST_Def_t* switch_struct){
	switch_struct->count.Enable = false;

	switch_struct->count.ms = 0;
}

/**
  * @brief  Call in timer ISR with 1ms to count each passed ms
  * @param	switch_struct		switch structure
  * @retval None
  */
void ST_countMillis(ST_Def_t* switch_struct){

	/* Count only if counter is enabled */
	if (switch_struct->count.Enable == true){
		switch_struct->count.ms++;
	}
}

/**
  * @brief  Read current switch value (after debouncing)
  * @param 	debounceTime  		switch debouncing time
  * @param	switch_struct		switch structure
  * @retval None
  */
void ST_readSwitchState(ST_Def_t* switch_struct, bool switchValue){
	switch_struct->State.current = switchValue;

	/* Processing based on new state */
	ST_processState(switch_struct);
}

/**
  * @brief  Return current switch level
  * @param	switch_struct		switch structure
  * @retval None
  */
uint32_t ST_getSwitchLevel(ST_Def_t* switch_struct){
	return switch_struct->Level;
}

/* Private function ---------------------------------------------------------*/
/**
  * @brief  Detect edge type (rissing/falling)
  * @param	switch_struct		switch structure
  * @retval None
  */
static void ST_detectEdgeType(ST_Def_t* switch_struct){

	/* If LOW to HIGH then RISING, if HIGH to LOW, then FALLING */
	if (switch_struct->State.previous == ST_STATE_LOW && switch_struct->State.current == ST_STATE_HIGH){
		switch_struct->Edge = ST_EDGE_RISING;
	}
	else if (switch_struct->State.previous == ST_STATE_HIGH && switch_struct->State.previous == ST_STATE_LOW) {
		switch_struct->Edge = ST_EDGE_FALLING;
	}
}

/**
  * @brief  Update switch level
  * @param	switch_struct		switch structure
  * @retval None
  */
static void ST_updateSwitchLevel(ST_Def_t* switch_struct){
	switch_struct->Level++;

	/* If Switch level is max, then set level to 1 */
	if (switch_struct->Level > ST_SWITCH_LEVEL_MAX){
		switch_struct->Level = 1;
	}
}

/**
  * @brief  Process the new switch state and related internal variables
  * @param	switch_struct		switch structure
  * @retval None
  */
static void ST_processState(ST_Def_t* switch_struct){

	/* Check if state has changed */
	if (switch_struct->State.current != switch_struct->State.previous){

		/* Detect if switch state is Rising/Falling */
		ST_detectEdgeType(switch_struct);

		ST_updateSwitchLevel(switch_struct);
	}
}
