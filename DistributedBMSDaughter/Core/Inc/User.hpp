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

/** Live Expressions / debugger monitor — updated every ~250 ms in StartVoltageTask. */
extern volatile uint16_t g_dbg_cell_mV[6];
extern volatile uint16_t g_dbg_ntc_adc[6];
extern volatile float g_dbg_ntc_C[6];
extern volatile uint16_t g_dbg_avg_cell_mV;
extern volatile uint16_t g_dbg_high_cell_mV;
extern volatile uint16_t g_dbg_low_cell_mV;
extern volatile float g_dbg_avg_C;
extern volatile float g_dbg_high_C;
extern volatile float g_dbg_low_C;
extern volatile uint8_t g_dbg_high_cell_idx;
extern volatile uint8_t g_dbg_high_temp_idx;
extern volatile uint8_t g_dbg_low_temp_idx;
extern volatile uint16_t g_dbg_can_id;
extern volatile uint32_t g_dbg_fault_mask;
extern volatile uint32_t g_dbg_cycle_count;
extern volatile uint32_t g_dbg_can_tx_ok;
extern volatile uint32_t g_dbg_can_esr;
extern volatile uint8_t g_dbg_can_state;  /* CanBus::State: 0 Init, 1 Healthy, 2 BusOff, 3 Recovering */

