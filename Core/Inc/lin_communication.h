
/******************************************************************************
  * @file    lin_communication.h
  * @author  Peter Pogacnik
  * @brief   LIN communication module header file.
  ****************************************************************************/


#ifndef INC_LIN_COMMUNICATION_H_
#define INC_LIN_COMMUNICATION_H_

#ifdef __cplusplus

 extern "C" {
#endif /* __cplusplus */

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

 /* Defines -------------------------------------------------------------------*/
/* LIN sizes */
#define LIN_DATA_SIZE		2					/* Specify LIN data size (up to 8 bytes) */
#define LIN_BUFF_SIZE		LIN_DATA_SIZE + 3	/* Buffer size (Sync + Id + Data + Checksum) */

 /* TypeDefs ------------------------------------------------------------------*/
 /**
   * @brief  Recieved buffer
  */

 typedef struct LIN_RecieveBuff {
    uint8_t 	Sync;					/* Sync byte */
    uint8_t 	Id;						/* Identifier */
    uint8_t 	Data[LIN_DATA_SIZE];	/* Data */
    uint8_t 	Checksum;				/* Checksum at the end of data */
 } LIN_DataFormat_t;

 /**
   * @brief  Lin communication general definition
  */

 typedef struct LIN_Def {
	bool					flgNewData;		/* Flag for new data */
	uint8_t					slaveId;			/* Id specified to current slave */
	LIN_DataFormat_t	 	Data;				/* LIN data */
	LIN_DataFormat_t	 	buffer;				/* Recieved data buffer */
 } LIN_Def_t;

 /* Private macros -----------------------------------------------------------*/
 /** @brief  LIN ID P0 parity check (P0 = ID0 XOR ID1 XOR ID2 XOR ID4) .
   * @param  ID byte.
   * @retval TRUE/FALSE
   */
 #define LIN_IS_VALID_ID_PARITY_P0(ID)         (((ID & 0x40) >> 6) == (((ID & 0x01) >> 0) ^ \
		 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	   ((ID & 0x02) >> 1) ^ \
																	   ((ID & 0x04) >> 2) ^ \
																	   ((ID & 0x10) >> 4)))

 /** @brief  LIN ID P1 parity check (P1 = ! (ID1 XOR ID3 XOR ID4 XOR ID5)) .
   * @param  ID byte.
   * @retval TRUE/FALSE
   */
#define LIN_IS_VALID_ID_PARITY_P1(ID)         (((ID & 0x80) >> 7) == !(((ID & 0x02) >> 1) ^ \
																  	   ((ID & 0x08) >> 3) ^ \
																	   ((ID & 0x10) >> 4) ^ \
																	   ((ID & 0x20) >> 5)))

 /* Function prototypes------------------------------------------------*/
 void LIN_Init_SlaveID(LIN_Def_t* lin_comm, uint8_t slaveId);
 uint8_t* LIN_getpBuff(LIN_Def_t* lin_comm);
 void LIN_processBuffer(LIN_Def_t* lin_comm);
 uint8_t* LIN_getpData(LIN_Def_t* lin_comm);
 bool LIN_isNewData(LIN_Def_t* lin_comm);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_LIN_COMMUNICATION_H_ */
