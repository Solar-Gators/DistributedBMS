/*
 * User.hpp
 *
 *  Created on: Aug 22, 2025
 *      Author: samrb
 */
#include "main.h"
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"

//#include "BQ76920.hpp"

//pull data tyes from main.cpp
extern I2C_HandleTypeDef hi2c2;
extern CAN_HandleTypeDef hcan1;
extern ADC_HandleTypeDef hadc1;
extern const osMutexAttr_t BMS_Mutex_attr;
extern osMutexId_t bmsMutex_id;
void setup();
