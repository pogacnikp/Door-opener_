/**
  ******************************************************************************
  * @file    flash_fnc.c
  * @author  Peter Pogacnik
  * @brief   Flash function to read/write into flash
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "flash_fnc.h"
/* Private function prototypes------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Write 8 bytes of data into flash
  * @param 	writeAddress  		address to write data
  * @param 	writeData  			data to be written
  * @retval None
  */
void FLASH_write8Bytes(uint32_t writeAddress, uint64_t writeData){
	HAL_FLASH_Unlock();
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, writeAddress, writeData) != HAL_OK){
		HAL_FLASH_GetError();
	}
	HAL_FLASH_Lock();
}

/**
  * @brief  Read 8 bytes of data from FLASH
  * @param 	readAddress  		data adrres in FLASH
  * @retval None
  */
uint64_t FLASH_read8Bytes(uint32_t readAddress){
	return *(uint64_t*) readAddress;
}

/* Private function ---------------------------------------------------------*/
