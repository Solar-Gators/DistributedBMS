/*
 * User.hpp
 *
 *  Created on: Nov 26, 2025
 *      Author: samrb
 */

#ifndef INC_USER_HPP_
#define INC_USER_HPP_

#include "main.h"
#include "cmsis_os.h"
#include "stm32g4xx_hal.h"

void setup();
/** Create RTOS mutexes and wire them into drivers; call after osKernelInitialize(), before osKernelStart(). */
void UserInitRtosSync(void);

void StartDefaultTask(void* argument);      // CAN RX + dispatch
void StartSafetyTask(void* argument);       // BmsManager update
void StartFleetTask(void* argument);        // Fleet aggregation
void StartVehicleTxTask(void* argument);    // Vehicle CAN interface update
void StartAuxTask(void* argument);          // ADS131M02 background sample



#endif /* INC_USER_HPP_ */
