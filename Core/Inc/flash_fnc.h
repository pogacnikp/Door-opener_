
/******************************************************************************
  * @file    flash_fnc.h
  * @author  Peter Pogacnik
  * @brief   Flash functions module header file.
  ****************************************************************************/

#ifndef INC_FLASH_FNC_H_
#define INC_FLASH_FNC_H_

#ifdef __cplusplus

 extern "C" {
#endif /* __cplusplus */

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32g0xx_hal.h"

 /* Defines -------------------------------------------------------------------*/
 /* TypeDefs ------------------------------------------------------------------*/
 /* Function prototypes------------------------------------------------*/
 void FLASH_write8Bytes(uint32_t writeAddress, uint64_t writeData);
 uint64_t FLASH_read8Bytes(uint32_t readAddress);

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* INC_FLASH_FNC_H_ */
