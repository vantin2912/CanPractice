/*
 * CanCom.c
 *
 *  Created on: Sep 20, 2022
 *      Author: vanti
 */
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#include "CANCom.h"



#define CANTransmitID	0x012
#define CANRcvID		0x0A2

uint8_t crcTable[256];

uint8_t CAN_FrameData[8]={0};


osMutexId_t RcvDataMutex;

const osMutexAttr_t RcvDataMutex_attr = {
  "RcvDataMutex",     // human readable mutex name
  osMutexRecursive | osMutexPrioInherit,    // attr_bits
  NULL,                // memory for control block
  0U                   // size for control block
};


void CRCInit(void) {
	uint8_t _crc;
	for (int i = 0; i < 0x100; i++) {
		_crc = i;

		for (uint8_t bit = 0; bit < 8; bit++) _crc = (_crc & 0x80) ? ((_crc << 1) ^ 0x1D) : (_crc << 1);

		crcTable[i] = _crc;
	}
}
uint8_t CalcCRC(uint8_t * buf, uint8_t len) {
        const uint8_t * ptr = buf;
        uint8_t _crc = 0xFF;

        while(len--) _crc = crcTable[_crc ^ *ptr++];

        return ~_crc;
}

static void CAN_ConfigFilter(CAN_HandleTypeDef* hcan)
{
	CAN_FilterTypeDef Filter;
	Filter.FilterActivation = CAN_FILTER_ENABLE;
	Filter.FilterIdHigh = CANRcvID<<5;
	Filter.FilterMode = CAN_FILTERMODE_IDLIST;
	Filter.FilterScale = CAN_FILTERSCALE_16BIT;
	Filter.FilterBank = 1;
	Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	Filter.SlaveStartFilterBank =20;
	HAL_CAN_ConfigFilter(hcan, &Filter);

}

void CAN_Config(CAN_HandleTypeDef* hcan)
{
	CRCInit();
	CAN_ConfigFilter(hcan);
	RcvDataMutex = osMutexNew(&RcvDataMutex_attr);

	HAL_CAN_Start(hcan);
}

void CAN_Receive(CAN_HandleTypeDef* hcan)
{
	HAL_StatusTypeDef Status;
	CAN_RxHeaderTypeDef Can_Header = {0};
	uint32_t FreeLevel;
	FreeLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_FILTER_FIFO0);
	uint8_t RcvBuffer[8]={0};
	while(FreeLevel > 0)
	{
		Status = HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &Can_Header, RcvBuffer);
		if(Status != HAL_OK) return;

//		MyPrintf("Practice Rcv ID = %.8lx Len = %ld \r\n", Can_Header.StdId, Can_Header.DLC);
		for(uint8_t i = 0; i< Can_Header.DLC ;++i)
		{
			MyPrintf("%d ", RcvBuffer[i]);
		}
		MyPrintf("\r\n");
		if(Can_Header.StdId == CANRcvID)
		{
			osMutexAcquire(RcvDataMutex, 0);
			memcpy(CAN_FrameData, RcvBuffer,2);
			osMutexRelease(RcvDataMutex);
		}
		FreeLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_FILTER_FIFO0);
	}
}

void CAN_Transmit(CAN_HandleTypeDef* hcan)
{
	CAN_TxHeaderTypeDef Can_Header = {0};
	HAL_StatusTypeDef Status;
	uint32_t Mailbox = 0;
	uint32_t FreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	if(FreeLevel == 0) return;

	uint8_t TransmitBuffer[8]= {0};
	osMutexAcquire(RcvDataMutex, 0);
	memcpy(TransmitBuffer,CAN_FrameData,2);
	osMutexRelease(RcvDataMutex);

	TransmitBuffer[2] = TransmitBuffer[0] + TransmitBuffer[1];
	TransmitBuffer[7] = CalcCRC(TransmitBuffer, 7);

	Can_Header.DLC = 8;
	Can_Header.StdId = CANTransmitID;


	Status = HAL_CAN_AddTxMessage(hcan, &Can_Header, TransmitBuffer, &Mailbox);

	if(Status == HAL_OK)
	{
//		printf("Message in mailbox %ld \r\n", Mailbox);
	}
	else
	{
		MyPrintf("Transmit Error Status %d Code = 0x%.8lx \r\n", Status, hcan->ErrorCode);
	}
}


