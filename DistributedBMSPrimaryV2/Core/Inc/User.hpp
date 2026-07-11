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
#include "BmsFleet.hpp"

void setup();
/** Create RTOS mutexes and wire them into drivers; call after osKernelInitialize(), before osKernelStart(). */
void UserInitRtosSync(void);

void StartDefaultTask(void* argument);      // CAN RX + dispatch
void StartSafetyTask(void* argument);       // BmsManager update
void StartFleetTask(void* argument);        // Fleet aggregation
void StartVehicleTxTask(void* argument);    // Vehicle CAN interface update
void StartAuxTask(void* argument);          // ADS131M02 background sample

/** Primary debugger: per-daughter averages from fleet (Live Expressions). */
extern volatile uint32_t g_dbg_can_rx_frames;
extern volatile uint32_t g_dbg_fleet_cycle;
extern volatile uint8_t g_dbg_daughter_online[MAX_MODULES];
extern volatile uint16_t g_dbg_daughter_avg_mV[MAX_MODULES];
extern volatile float g_dbg_daughter_avg_C[MAX_MODULES];
extern volatile float g_dbg_daughter_low_C[MAX_MODULES];
extern volatile uint16_t g_dbg_daughter_low_mV[MAX_MODULES];
extern volatile uint16_t g_dbg_daughter_high_mV[MAX_MODULES];
/** Fleet-wide cell extremes (from BmsFleet summary, used for imbalance fault). */
extern volatile uint16_t g_dbg_fleet_highest_mV;
extern volatile uint16_t g_dbg_fleet_lowest_mV;
extern volatile uint16_t g_dbg_fleet_imbalance_mV;
extern volatile uint8_t g_dbg_fleet_highest_cell_idx;
extern volatile uint8_t g_dbg_fleet_lowest_cell_idx;
/** ms since last decoded frame; >500 means stale cleared online. 0 = never decoded. */
extern volatile uint32_t g_dbg_daughter_age_ms[MAX_MODULES];
extern volatile uint32_t g_dbg_daughter_last_update_ms[MAX_MODULES];
extern volatile uint16_t g_dbg_last_rx_id;
extern volatile uint8_t g_dbg_last_rx_type;
extern volatile int32_t g_dbg_last_rx_slot;
extern volatile uint32_t g_dbg_rx_unknown_id;
extern volatile uint32_t g_dbg_rx_decode_ok;
extern volatile uint32_t g_dbg_rx_decode_fail;
extern volatile uint32_t g_dbg_fdcan2_fifo_level;
extern volatile uint32_t g_dbg_fdcan2_psr;
extern volatile uint16_t g_dbg_active_faults;
extern volatile uint8_t g_dbg_bms_state;
/** Pack (DHAB S/134 → ADS1115 AIN0/AIN1) / aux+fan (INA226), updated in safety task. */
extern volatile float g_dbg_pack_current_A;
extern volatile float g_dbg_pack_current_adc_V;
extern volatile float g_dbg_pack_current_adc_low_V;
extern volatile float g_dbg_pack_current_adc_high_V;
extern volatile float g_dbg_soc_percent;
extern volatile float g_dbg_aux_current_A;
extern volatile float g_dbg_fan_current_A;
extern volatile float g_dbg_aux_shunt_mV;
extern volatile float g_dbg_aux_bus_V;
extern volatile float g_dbg_fan_shunt_mV;
extern volatile float g_dbg_fan_bus_V;
extern volatile uint8_t g_dbg_ina_aux_init_ok;
extern volatile uint16_t g_dbg_ina_aux_manuf_id;
extern volatile uint32_t g_dbg_ina_aux_read_ok;
extern volatile uint32_t g_dbg_ina_aux_read_fail;
extern volatile uint32_t g_dbg_ina_aux_last_hal;

#endif /* INC_USER_HPP_ */
