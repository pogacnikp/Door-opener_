/**
  ******************************************************************************
  * @file    lin_communication.c
  * @author  Peter Pogacnik
  * @brief   LIN communication functions
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lin_communication.h"

/* Private function prototypes------------------------------------------------*/
static bool LIN_isIDValid(LIN_Def_t* lin_comm);
static bool LIN_isBuffDataValid(LIN_Def_t* lin_comm);

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Initialize slave ID
  * @param 	slaveId  		Slave ID for communication
  * @param	lin_comm		LIN communication structure
  * @retval None
  */
void LIN_Init_SlaveID(LIN_Def_t* lin_comm, uint8_t slaveId){
	/* ID values above 59 are reserved */
	if (slaveId <= 59) {
		lin_comm->slaveId = slaveId;
	} else {
		//error
	}
}

/**
  * @brief  Get pointer on buffer n
  * @param	lin_comm			LIN communication structure
  * @retval Pointer to buffer
  */
uint8_t* LIN_getpBuff(LIN_Def_t* lin_comm){
	return (uint8_t*) &lin_comm->buffer;
}

/**
  * @brief  Process buffer data and write into data struct
  * @param	lin_comm			LIN communication structure
  * @retval None
  */
void LIN_processBuffer(LIN_Def_t* lin_comm){
	/* Check if recived ID is valid */
	if (LIN_isIDValid(lin_comm) == true){

		/* Check if recieved data is valid */
		if (LIN_isBuffDataValid(lin_comm) == true){
			/* Write data from buffer to data struct */
			lin_comm->Data.Sync = lin_comm->buffer.Sync;
			lin_comm->Data.Id = lin_comm->buffer.Id;
			for (size_t i = 0; i < LIN_DATA_SIZE; i++){
				lin_comm->Data.Data[i] = lin_comm->buffer.Data[i];
			}
			lin_comm->Data.Checksum = lin_comm->buffer.Checksum;

			/* Set new data flag */
			lin_comm->flgNewData = true;
		} else {
			lin_comm->flgNewData = false;
		}

	} else {
		lin_comm->flgNewData = false;
	}
}

/**
  * @brief  Get pointer on data from LIN communication
  * @param	lin_comm			LIN communication structure
  * @retval Pointer of LIN data
  */
uint8_t* LIN_getpData(LIN_Def_t* lin_comm){
	return (uint8_t*) &lin_comm->Data.Data;
}

/**
  * @brief  Check if Data is received
  * @param	lin_comm			LIN communication structure
  * @retval TRUE/FALSE
  */
bool LIN_isNewData(LIN_Def_t* lin_comm){
	return &lin_comm->flgNewData;
}

/* Private function ---------------------------------------------------------*/
/**
  * @brief  Check if ID parity and
  * @param	lin_comm			LIN communication structure
  * @retval None
  */
static bool LIN_isIDValid(LIN_Def_t* lin_comm){
	/* Check ID parity bits */
	if (LIN_IS_VALID_ID_PARITY_P0(lin_comm->buffer.Id) && LIN_IS_VALID_ID_PARITY_P1(lin_comm->buffer.Id)) {
		/* Check if recieved ID is the same  as initialized slave ID */
		if ((lin_comm->buffer.Id & 0x3F) == lin_comm->slaveId){
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

/**
  * @brief  Check if buffer data is according to checksum
  * @param	lin_comm			LIN communication structure
  * @retval None
  */
static bool LIN_isBuffDataValid(LIN_Def_t* lin_comm){
	uint16_t sumOfData = 0;
	uint8_t checksum = 0;

	/*
	 * Classic LIN checksum calculation:
	 * The checksum contains the inverted eight bit
	 * sum with carry over all data bytes
	 * */
	for (size_t i = 0; i< LIN_DATA_SIZE; i++) {
		sumOfData += lin_comm->buffer.Data[i];

		if (sumOfData > UINT8_MAX){
			sumOfData = UINT8_MAX - sumOfData;
		}
	}
	checksum = sumOfData;

	/* At the end invert */
	checksum ^= UINT8_MAX;

	/* If checksums are the same, then data is valid */
	return checksum == lin_comm->buffer.Checksum;
}

