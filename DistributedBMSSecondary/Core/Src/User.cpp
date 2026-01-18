/*
 * User.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: samrb
 */
#include "User.hpp"

#include "CanBus.hpp"
#include "BmsFleet.hpp"

#include <stdint.h>
#include <stddef.h>

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

static CanBus can(hcan1);

static BmsFleet fleet;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (auto* cb = CanBus::isr_instance()) {
        cb->onRxFifo0Pending();
    }
}

//Setup Function
void setup(){

	can.configureFilterAcceptAll();  // or configureFilterStdMask(0x123, 0x7FF);
	can.start();

	fleet.registerDaughter(0x100, 0);
	fleet.registerDaughter(0x101, 1);

}

void StartDefaultTask(void *argument)
{
    uint8_t txbuf[64];

    static uint8_t next_module = 0;

    for (;;)
    {
        // -----------------------------------------------------------------
        // Update fleet aggregation
        // -----------------------------------------------------------------
        fleet.processModules();

        // -----------------------------------------------------------------
        // Send fleet-level summary (every cycle)
        // -----------------------------------------------------------------
        size_t len = fleet.packFleetData(txbuf);
        if (len > 0) {
            HAL_UART_Transmit(&huart2, txbuf, len, 10000);
        }

        // -----------------------------------------------------------------
        // Send ONE module summary (round-robin)
        // -----------------------------------------------------------------
        /*
        for (uint8_t i = 0; i < MAX_MODULES; i++) {
            uint8_t idx = (next_module + i) % MAX_MODULES;

            if (fleet.isOnline(idx)) {
                size_t mlen = fleet.packModuleData(txbuf, idx);
                if (mlen > 0) {
                    HAL_UART_Transmit(&huart2, txbuf, mlen, 10000);
                }

                // Advance round-robin pointer
                next_module = (idx + 1) % MAX_MODULES;
                break;
            }
        }
		*/
        // -----------------------------------------------------------------
        // UART pacing
        // -----------------------------------------------------------------



        osDelay(250);
    }
}


void StartCanTask(void *argument){

	while(1){
		CanBus::Frame rx;
		if (can.read(rx)) {
			fleet.handleMessage(rx, HAL_GetTick());
			HAL_GPIO_TogglePin(OK_GPIO_Port, OK_Pin);
		}
		osDelay(10);
	}

}
