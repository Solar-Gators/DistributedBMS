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
#include "CanBus.hpp"
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

//Intilize BQ chip driver
BQ7692000PW bq(&hi2c2);
//Initilize BMS data driver
BMS bms(DeviceConfig::CELL_COUNT_CONF);
//Initlize CAN driver
static CANDriver::CANDevice can(&hcan1);

FaultManager faultManager;

static int count = 0;
HAL_StatusTypeDef allCallback(const CANDriver::CANFrame& msg, void* ctx){
	count++;
	return HAL_OK;
}

//ADC dma stuff
bool TempDMAComplete;
float adc_vals[ADC_NUM_CONVERSIONS];
volatile uint16_t adc_buf[ADC_NUM_CONVERSIONS];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//move data between buffer arrays
	TempDMAComplete = true;
}


static uint16_t packVoltage = 0;
static std::array<uint16_t, CELL_COUNT> cellVoltages{};

static uint16_t dieTemp = 0;
static std::array<uint16_t, CELL_COUNT> cellTempADC{};

bool debugMode;

void setup(){

	bmsMutex_id = osMutexNew(&BMS_Mutex_attr);
	can.addCallbackAll(allCallback);
	can.StartCANDevice();

	debugMode = true;

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{

	CANDriver::CANFrame avg_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

	HAL_GPIO_WritePin(TS1_GPIO_Port, TS1_Pin, GPIO_PIN_SET);

	if(debugMode == false){
	//initalize BMS stuff
		while (bq.init() != HAL_OK)
		{
			// Error handling
			HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_RESET);

		// Configure fault manager restart settings
		faultManager.setRestartEnabled(true);
		faultManager.setRestartDelay(5000);  // 5 seconds delay between restart attempts
		faultManager.setMaxRestartAttempts(3);  // Maximum 3 restart attempts
		faultManager.setCommunicationTimeout(10000);  // 10 seconds communication timeout

		// Initialize communication time to prevent immediate timeout on startup
		faultManager.updateCommunicationTime(HAL_GetTick());
	}
	TempDMAComplete = false;

	for(;;)
	{
		if(debugMode == false){
			osMutexAcquire(bmsMutex_id, osWaitForever);
			auto& r = bms.results();
			osMutexRelease(bmsMutex_id);

			// Build message
			auto avg_data = CanFrames::make_average_stats(r);

			avg_msg.LoadData(avg_data.bytes.data(), 8);
			can.Send(&avg_msg);

			uint32_t esr = hcan1.Instance->ESR;
			// uint32_t tec = (esr >> 16) & 0xFF;
			// uint32_t rec = (esr >> 24) & 0xFF;
			// uint32_t lec =  esr & 0x7;

			if (0 == esr)
			{
			  faultManager.updateCommunicationTime(HAL_GetTick());
			  faultManager.setFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR, false);
			  // TODO: May have to manually reset esr TBD
			}
			else
			{
			  faultManager.setFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR, true);
			}

			faultManager.update(HAL_GetTick());

			// Check if restart is needed
			if (faultManager.shouldRestart()) {
			  faultManager.attemptRestart();
			  // Perform system restart - this will reset the microcontroller
			  HAL_NVIC_SystemReset();
			}

			if (faultManager.hasActiveFaults()) {
			  HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_SET);
			} else {
			  HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_RESET);
			}

			if(faultManager.isSystemFunctional()) {
			  HAL_GPIO_TogglePin(GPIOB, OK_Pin);
			}

			osDelay(DeviceConfig::CYCLE_TIME_MS);
		}else{
			HAL_GPIO_TogglePin(GPIOB, OK_Pin);
			osDelay(DeviceConfig::CYCLE_TIME_MS);
		}
  }
  /* USER CODE END 5 */
}



void StartVoltageTask(void *argument)
{

	bool voltage_read_success = true;
	CANDriver::CANFrame highvolt_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

	for (;;)
	{
		if(debugMode == false){
			osMutexAcquire(bmsMutex_id, osWaitForever);
			if (bq.getBAT(&packVoltage) != HAL_OK)
			{
				faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
				voltage_read_success = false;
			}
			else
			{
				faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			}

			if (bq.getVC(cellVoltages) != HAL_OK)
			{
				faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
				voltage_read_success = false;
			}
			else
			{
				faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			}

			if (voltage_read_success)
			{
				bms.set_cell_mV(cellVoltages);
				bms.update();
			}
			osMutexRelease(bmsMutex_id);

			osDelay(10);

			osMutexAcquire(bmsMutex_id, osWaitForever);
			auto& r = bms.results();
			osMutexRelease(bmsMutex_id);

			auto data = CanFrames::make_voltage_extremes(r);
			highvolt_msg.LoadData(data.bytes.data(), 8);
			can.Send(&highvolt_msg);

			osDelay(DeviceConfig::CYCLE_TIME_MS);
		}else{
			osDelay(DeviceConfig::CYCLE_TIME_MS);
		}
	}
}

void StartTemperatureTask(void *argument)
{

    CANDriver::CANFrame hightemp_msg(DeviceConfig::CAN_ID, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

    for (;;)
    {
    	if(debugMode == false){
			bool temp_read_success = true;   // <-- reset every loop

			//external temperatures
			TempDMAComplete = false;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc_buf, ADC_NUM_CONVERSIONS);
			while (!TempDMAComplete) {}

			for (uint8_t i = 0; i < ADC_NUM_CONVERSIONS; i++) {
				cellTempADC[i] = adc_buf[i];

				if (adc_buf[i] == 0 || adc_buf[i] == 4095) {
					faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, true);
					temp_read_success = false;
				} else {
					faultManager.clearFault(FaultManager::FaultType::ADC_READ_ERROR);
				}
			}

			osMutexAcquire(bmsMutex_id, osWaitForever);
			if (bq.getDieTemp(&dieTemp) != HAL_OK) {
				faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
				temp_read_success = false;
			} else {
				faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			}

			if (temp_read_success) {
				bms.set_ntc_counts(cellTempADC);
				bms.update();
			}
			osMutexRelease(bmsMutex_id);

			osDelay(10);

			osMutexAcquire(bmsMutex_id, osWaitForever);
			auto& r = bms.results();
			osMutexRelease(bmsMutex_id);

			auto data = CanFrames::make_high_temp(r);
			hightemp_msg.LoadData(data.bytes.data(), 8);
			can.Send(&hightemp_msg);

			osDelay(DeviceConfig::CYCLE_TIME_MS * 4);
    	}else{
    		osDelay(DeviceConfig::CYCLE_TIME_MS * 4);
    	}
    }
}


