/*
 * User.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: samrb
 */
#include "User.hpp"

//#include "CanDriver.hpp"
#include "CanBus.hpp"

#include "BmsFleet.hpp"
//include "UartFleetPack.hpp"

#include <stdint.h>
#include <stddef.h>

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

//static CANDriver::CANDevice can(&hcan1);
static CanBus can(hcan1);

static BmsFleet fleet;

/*
HAL_StatusTypeDef allCallback(const CANDriver::CANFrame& msg, void* ctx){
	HAL_GPIO_TogglePin(OK_GPIO_Port, OK_Pin);
	fleet.handle(msg, HAL_GetTick());
	return HAL_OK;
}

HAL_StatusTypeDef daughterOneCallback(const CANDriver::CANFrame& msg, void* ctx){
	HAL_GPIO_TogglePin(OK_GPIO_Port, OK_Pin);
	fleet.handle(msg, HAL_GetTick());
	return HAL_OK;
}
*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (auto* cb = CanBus::isr_instance()) {
        cb->onRxFifo0Pending();
    }
}

//Setup Function
void setup(){
	//can.AddFilterRange(0x101, 4, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 0);
	//can.addCallbackRange(0x101, 4, SG_CAN_ID_STD, daughterOneCallback, NULL);
	//can.AddFilterId(0x101, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 0);
	//can.addCallbackId(0x101, SG_CAN_ID_STD, daughterOneCallback, NULL);

	// Add Daughters 2-6 here

	//can.addCallbackAll(allCallback);
	can.configureFilterAcceptAll();  // or configureFilterStdMask(0x123, 0x7FF);
	can.start();

	//fleet.register_node(0x101, 0);


	//can.StartCANDevice();
}



static volatile uint32_t g_lastHeartbeatInterval = 0;

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	//uint8_t txbuf[64];  // Buffer for encoded frame (SOF + LEN + payload + CRC)
	//static uint8_t module_cursor = 0;
	static uint32_t last_heartbeat_ms = 0;
	//tatic uint32_t heartbeat_counter = 0;
	//static uint8_t frame_rotation = 0;  // 0=fleet, 1=module, 2=heartbeat
	uint32_t now = 0;

	for(;;)
	{
		now = osKernelGetTickCount();

		if (last_heartbeat_ms == 0U) {
			last_heartbeat_ms = now;
		}
		/*
		bool have_data = false;
		for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
			if (fleet.has_any_data(i)) {
				have_data = true;
				break;
			}
		}

		// Rotate between frame types to avoid overwhelming the receiver
		switch (frame_rotation) {


		case 0:  // Module summary
			if (have_data) {
				for (uint8_t attempts = 0; attempts < BmsFleetCfg::MAX_MODULES; ++attempts) {
					uint8_t idx = module_cursor;
					module_cursor = (uint8_t)((module_cursor + 1) % BmsFleetCfg::MAX_MODULES);
					if (!fleet.has_any_data(idx)) {
						continue;
					}

					size_t mod_len = uart_make_module_summary(fleet, idx, now, txbuf, sizeof(txbuf));
					if (mod_len > 0) {
						HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, txbuf, mod_len, 1000);
						if (status != HAL_OK) {
							// TODO: handle transmission error
						} else {
							// Give receiver time to process frame
							osDelay(50);
						}
						break;
					}
				}
			}
			frame_rotation = 1;
			break;

		case 1:  // Heartbeat
			if ((now - last_heartbeat_ms) >= 1000U) {
				uint32_t interval = now - last_heartbeat_ms;
				g_lastHeartbeatInterval = interval;
				HAL_GPIO_TogglePin(GPIOB, ERROR_Pin);
				size_t hb_len = uart_make_heartbeat(heartbeat_counter++, txbuf, sizeof(txbuf));
				if (hb_len > 0) {
					HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, txbuf, hb_len, 1000);
					if (status != HAL_OK) {
						// TODO: handle transmission error
					} else {
						// Give receiver time to process frame
						osDelay(50);
					}
				}
				last_heartbeat_ms = now;
			}
			frame_rotation = 0;
			break;
		}
		*/
		osDelay(250);
	}
  /* USER CODE END 5 */
}

void StartCanTask(void *argument){

	while(1){
		CanBus::Frame rx;
		if (can.read(rx)) {
			//fleet.handle(rx, HAL_GetTick());
			HAL_GPIO_TogglePin(OK_GPIO_Port, OK_Pin);
		}
	}
}
