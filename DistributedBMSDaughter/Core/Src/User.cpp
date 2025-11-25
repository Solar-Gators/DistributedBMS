/*
 * User.cpp
 *
 *  Created on: Aug 22, 2025
 *      Author: samrb
 */
# define ADC_NUM_CONVERSIONS 5

#include "User.hpp"

//Lower Level Drivers
#include "BQ7692000.hpp"
#include "CanDriver.hpp"

//Data handlers
#include "BMS.hpp"
#include "CanFrame.cpp"
#include "FaultManager.hpp"
#include "DeviceConfig.hpp"

//C++ stuff
#include <array>
#include <cstring>
#include <cstdio>
#include <cstdint>


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

//Can Callback
static int count = 0;
HAL_StatusTypeDef allCallback(const CANDriver::CANFrame& msg, void* ctx){
	count++;
	return HAL_OK;
}

//ADC DMA stuff
bool TempDMAComplete;
volatile uint16_t adc_buf[ADC_NUM_CONVERSIONS];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//move data between buffer arrays
	TempDMAComplete = true;
}

//BQ Chip Data types
static uint16_t packVoltage = 0;
static std::array<uint16_t, CELL_COUNT> cellVoltages{};
//Temp Data
static uint16_t dieTemp = 0;
static std::array<uint16_t, CELL_COUNT> cellTempADC{};

bool debugMode;

void setup(){

	bmsMutex_id = osMutexNew(&BMS_Mutex_attr);
	can.addCallbackAll(allCallback);
	can.StartCANDevice();

	debugMode = true;

}

//Get Data
void StartDefaultTask(void *argument)
{

	HAL_GPIO_WritePin(TS1_GPIO_Port, TS1_Pin, GPIO_PIN_SET);

	while (bq.init() != HAL_OK)
	{
		// Error handling
		HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_RESET);

	bool temp_read_success = true;
	bool voltage_read_success = true;

	for(;;)
	{

		osMutexAcquire(bmsMutex_id, osWaitForever);
		if (bq.getBAT(&packVoltage) != HAL_OK)
		{
			faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
			voltage_read_success = false;
		}
		else
		{
			faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			voltage_read_success = true;
		}
		if (bq.getVC(cellVoltages) != HAL_OK)
		{
			faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
			voltage_read_success = false;
		}
		else
		{
			faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			voltage_read_success = true;
		}

		if (voltage_read_success)
		{
			bms.set_cell_mV(cellVoltages);
		}
		osMutexRelease(bmsMutex_id);


		TempDMAComplete = false;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc_buf, ADC_NUM_CONVERSIONS);
		while (!TempDMAComplete) {}

		for (int i = 0; i < ADC_NUM_CONVERSIONS; i++) cellTempADC[i] = adc_buf[i];

		if (temp_read_success) {
			bms.set_ntc_counts(cellTempADC);
		}

		osDelay(DeviceConfig::CYCLE_TIME_MS);

  }
  /* USER CODE END 5 */
}


//Send Data
void StartVoltageTask(void *argument)
{

	CANDriver::CANFrame avg_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);
	CANDriver::CANFrame highvolt_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);
	CANDriver::CANFrame hightemp_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

	for (;;)
	{

		osMutexAcquire(bmsMutex_id, osWaitForever);
		bms.update();
		auto& r = bms.results();
		osMutexRelease(bmsMutex_id);

		auto data1 = CanFrames::make_average_stats(r);
		avg_msg.LoadData(data1.bytes.data(), 8);
		can.Send(&avg_msg);

		auto data2 = CanFrames::make_voltage_extremes(r);
		highvolt_msg.LoadData(data2.bytes.data(), 8);
		can.Send(&highvolt_msg);

		auto data3 = CanFrames::make_high_temp(r);
		hightemp_msg.LoadData(data3.bytes.data(), 8);
		can.Send(&hightemp_msg);

		osDelay(DeviceConfig::CYCLE_TIME_MS);

	}
}


