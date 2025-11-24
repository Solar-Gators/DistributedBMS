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

extern osMutexId_t bmsMutex_id;
extern const osMutexAttr_t BMS_Mutex_attr;

void setup();
