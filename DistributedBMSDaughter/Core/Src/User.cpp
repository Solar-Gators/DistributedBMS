/*
 * User.cpp
 *
 *  Created on: Aug 22, 2025
 *      Author: samrb
 */
# define CELLS 5

#include "User.hpp"

//Lower Level Drivers
#include "BQ7692000.hpp"

//Data handlers
#include "BMS.hpp"
#include "CanFrame.cpp"
#include "CanBus.hpp"
#include "FaultManager.hpp"
#include "DataValidator.hpp"
#include "DeviceConfig.hpp"

//C++ stuff
#include <array>
#include <cstring>
#include <cstdio>
#include <cstdint>



//hardware intialization
BQ7692000PW bq(&hi2c2);
static CanBus can1(hcan1);
//CANDriver::CANDevice can(&hcan1);

//Data handler Inits
BMS bms(DeviceConfig::CELL_COUNT_CONF);
FaultManager faultManager;
DataValidator dataValidator;





//ADC DMA stuff
volatile bool TempDMAComplete;
volatile uint16_t adc_buf[CELLS];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//move data between buffer arrays
	TempDMAComplete = true;
}

//BQ Chip Data types
static std::array<uint16_t, CELLS> cellVoltages{};
//Temp Data
static std::array<uint16_t, CELLS> cellTempADC{};

bool debugMode;

static void CanErrorCallback(CanBus& can, uint32_t err)
{
    if (err & CAN_ESR_BOFF) {
        faultManager.setFault(FaultManager::FaultType::CAN_BUS_OFF);
        return;
    }

    if (err & (CAN_ESR_EPVF | CAN_ESR_EWGF)) {
        faultManager.setFault(FaultManager::FaultType::CAN_ERROR_PASSIVE);
    }

    if (err == HAL_CAN_ERROR_NONE) {
        faultManager.clearFault(FaultManager::FaultType::CAN_ERROR_PASSIVE);
        faultManager.clearFault(FaultManager::FaultType::CAN_BUS_OFF);
    }
}
//Setup Function
void setup(){

	bmsMutex_id = osMutexNew(&BMS_Mutex_attr);

    can1.setErrorCallback(CanErrorCallback);
    can1.configureFilterAcceptAll();
    can1.start();

	debugMode = true;

}

//Collect voltage and temperature data (ignore task name)
void StartDefaultTask(void *argument)
{

	//Intalize BMS chip
	HAL_GPIO_WritePin(TS1_GPIO_Port, TS1_Pin, GPIO_PIN_SET);

	int retries = 0;
	const int MAX_RETRIES = 5;

	while (bq.init() != HAL_OK && retries < MAX_RETRIES) {
	    retries++;
	    faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
	    HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_SET);
	    osDelay(100); // or HAL_Delay if before scheduler
	}

	if (retries == MAX_RETRIES) {
	    // hard fault state? maybe stay here or signal via CAN
	} else {
		faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
	    HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_RESET);
	}

	for(;;)
	{

		osMutexAcquire(bmsMutex_id, osWaitForever);
		//Get Pack and cell voltages
		if (bq.getVC(cellVoltages) != HAL_OK)
		{
			faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
		}
		else
		{
			faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			if(dataValidator.validateCellVoltages(cellVoltages) == 0){
				bms.set_cell_mV(cellVoltages);
				faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			}else{
				faultManager.setFault(FaultManager::FaultType::BQ76920_RESULT_ERROR);
			}

		}
		osMutexRelease(bmsMutex_id);

		//read tempatures
		TempDMAComplete = false;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc_buf, CELLS);
		while (!TempDMAComplete) {}

		for (int i = 0; i < CELLS; i++) cellTempADC[i] = adc_buf[i];

		if(dataValidator.validateADCReadings(cellTempADC) == 0){
			bms.set_ntc_counts(cellTempADC);
			faultManager.clearFault(FaultManager::FaultType::ADC_RESULT_ERROR);
		}else{
			faultManager.setFault(FaultManager::FaultType::ADC_RESULT_ERROR);
		}

		bms.setFaults(faultManager.getFaultMask());

		osDelay(DeviceConfig::CYCLE_TIME_MS);

  }
  /* USER CODE END 5 */
}


void StartVoltageTask(void *argument)
{
    for (;;)
    {
        /* ---------------- Acquire BMS data ---------------- */
        osMutexAcquire(bmsMutex_id, osWaitForever);
        bms.update();
        const auto& r = bms.results();
        osMutexRelease(bmsMutex_id);

        /* ---------------- CAN housekeeping ---------------- */
        can1.poll();   // Required for recovery / error handling
        if (can1.busOffLatched()) {
            faultManager.setFault(FaultManager::FaultType::CAN_BUS_OFF);
        }else{
        	faultManager.clearFault(FaultManager::FaultType::CAN_BUS_OFF);
        }


        /* ---------------- Build CAN frames ---------------- */
        const auto avgStats     = CanFrames::make_average_stats(r);
        const auto voltExtremes = CanFrames::make_voltage_extremes(r);
        const auto highTemp     = CanFrames::make_high_temp(r);

        /* ---------------- Transmit frames ---------------- */
        bool tx_ok = true;

        tx_ok &= (can1.sendStd(DeviceConfig::CAN_ID,
                               avgStats.bytes,
                               avgStats.dlc) == CanBus::Result::Ok);

        tx_ok &= (can1.sendStd(DeviceConfig::CAN_ID,
                               voltExtremes.bytes,
                               voltExtremes.dlc) == CanBus::Result::Ok);

        tx_ok &= (can1.sendStd(DeviceConfig::CAN_ID,
                               highTemp.bytes,
                               highTemp.dlc) == CanBus::Result::Ok);

        /* ---------------- Fault handling ---------------- */
        if (!tx_ok) {
            faultManager.setFault(FaultManager::FaultType::CAN_TX_ERROR);
        } else {
            faultManager.clearFault(FaultManager::FaultType::CAN_TX_ERROR);


        }


        /* ---------------- Status LED ---------------- */
        if (faultManager.getFaultMask() == 0) {
            HAL_GPIO_TogglePin(GPIOB, OK_Pin);
        }

        osDelay(DeviceConfig::CYCLE_TIME_MS);
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CanBus::handleRxFifo0(hcan);
}

void HAL_CAN_BusOffCallback(CAN_HandleTypeDef* hcan) {
    CanBus::handleBusOff(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
    CanBus::handleError(hcan);
}


