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
#include "CanDriver.hpp"

//Data handlers
#include "BMS.hpp"
#include "CanFrame.cpp"
#include "FaultManager.hpp"
#include "DataValidator.hpp"
#include "DeviceConfig.hpp"

//C++ stuff
#include <array>
#include <cstring>
#include <cstdio>
#include <cstdint>

//pull data tyes from main.cpp
extern I2C_HandleTypeDef hi2c2;
extern CAN_HandleTypeDef hcan1;
extern ADC_HandleTypeDef hadc1;
extern const osMutexAttr_t BMS_Mutex_attr;
extern osMutexId_t bmsMutex_id;

//hardware intialization
BQ7692000PW bq(&hi2c2);
static CANDriver::CANDevice can(&hcan1);

//Data handler Inits
BMS bms(DeviceConfig::CELL_COUNT_CONF);
FaultManager faultManager;
DataValidator dataValidator;

//Can Callback
static int count = 0;
HAL_StatusTypeDef allCallback(const CANDriver::CANFrame& msg, void* ctx){
	count++;
	return HAL_OK;
}

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

//Setup Function
void setup(){

	bmsMutex_id = osMutexNew(&BMS_Mutex_attr);
	can.addCallbackAll(allCallback);
	can.StartCANDevice();

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

		osDelay(DeviceConfig::CYCLE_TIME_MS);

  }
  /* USER CODE END 5 */
}


//Send Data over CAN (ignore task name)
void StartVoltageTask(void *argument)
{

	//Initalize CAN frames
	CANDriver::CANFrame avg_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);
	CANDriver::CANFrame highvolt_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);
	CANDriver::CANFrame hightemp_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

	for (;;)
	{
		//Update BMS Values
		osMutexAcquire(bmsMutex_id, osWaitForever);
		bms.update();
		auto& r = bms.results();
		osMutexRelease(bmsMutex_id);

		//Send CAN frames
		auto data1 = CanFrames::make_average_stats(r);
		avg_msg.LoadData(data1.bytes.data(), 8);

		auto data2 = CanFrames::make_voltage_extremes(r);
		highvolt_msg.LoadData(data2.bytes.data(), 8);

		auto data3 = CanFrames::make_high_temp(r);
		hightemp_msg.LoadData(data3.bytes.data(), 8);

        bool allSuccessful = true;   // Track success for this cycle

        // ---- EXAMPLE TRANSMISSIONS ----
        if (can.Send(&avg_msg) != HAL_OK) {
            allSuccessful = false;
        }
        if (can.Send(&highvolt_msg) != HAL_OK) {
            allSuccessful = false;
        }
        if (can.Send(&hightemp_msg) != HAL_OK) {
            allSuccessful = false;
        }

        // Set or clear CAN fault based on the entire cycle:
        if (!allSuccessful) {
            faultManager.setFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR);
        } else {
            faultManager.clearFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR);
        }

        osDelay(DeviceConfig::CYCLE_TIME_MS);

	}
}
