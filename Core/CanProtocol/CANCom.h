/*
 * CanCom.h
 *
 *  Created on: Sep 20, 2022
 *      Author: vanti
 */

#ifndef CANCOM_H_
#define CANCOM_H_

#include "main.h"


void CAN_Config(CAN_HandleTypeDef* hcan);

void CAN_Receive(CAN_HandleTypeDef* hcan);

void CAN_Transmit(CAN_HandleTypeDef* hcan);

#endif /* CANCOM_H_ */
