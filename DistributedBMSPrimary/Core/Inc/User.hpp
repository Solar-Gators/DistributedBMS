/*
 * User.hpp
 *
 *  Created on: Jan 2025
 *      Author: samrb
 */

#ifndef INC_USER_HPP_
#define INC_USER_HPP_

#include "main.h"
#include "stm32l5xx_hal.h"

// Pull data types from main.cpp
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart4;

void setup();
void loop();

#endif /* INC_USER_HPP_ */

