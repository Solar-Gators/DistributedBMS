/*
 * User.cpp
 *
 *  Created on: Aug 22, 2025
 *      Author: samrb
 */
# define CELLS 6
# define ADC_CHANNEL_COUNT 7

/**
 * Set to 1 for bench / bring-up: fixed cell mV and NTC ADC counts (no VCOUT, no temp ADC).
 * Set to 0 for production: real AFE cell read + DMA NTC channels.
 */
#ifndef USE_STANDIN_SENSORS
#define USE_STANDIN_SENSORS 1
#endif

#include "User.hpp"

//Lower Level Drivers
// #include "BQ7692000.hpp"
#include "BQ76925PWR.hpp"

//Data handlers
#include "BMS.hpp"
#include "CanFrame.hpp"
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
BQ76925PWR bq(&hi2c2);
static CanBus can1(hcan1);
//CANDriver::CANDevice can(&hcan1);

//Data handler Inits
BMS bms(DeviceConfig::CELL_COUNT_CONF);
FaultManager faultManager;
DataValidator dataValidator;





//ADC DMA stuff

volatile bool TempDMAComplete;
volatile uint16_t adc_buf[ADC_CHANNEL_COUNT];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//move data between buffer arrays
	TempDMAComplete = true;
}

//BQ Chip Data types
static std::array<uint16_t, CELLS> cellVoltages{};
//Temp Data
static std::array<uint16_t, CELLS> cellTempADC{};

#if USE_STANDIN_SENSORS
/** Fixed per-cell voltages (mV) for primary fleet aggregation / CAN bring-up. */
static constexpr std::array<uint16_t, CELLS> kStandinCell_mV = {{
    3700u, 3710u, 3690u, 3720u, 3705u, 3695u,
}};
/**
 * 12-bit ADC counts into BMS::ntc_to_C() (divider: NTC top, 10k to GND).
 * ~2048 ≈ room temperature; small spread so high/avg differ slightly.
 */
static constexpr std::array<uint16_t, CELLS> kStandinNtcCounts = {{
    2048u, 2055u, 2040u, 2060u, 2035u, 2050u,
}};
#endif

bool debugMode;

#if !USE_STANDIN_SENSORS
// Real VCOUT + ADC (used when USE_STANDIN_SENSORS is 0).
static HAL_StatusTypeDef readCellsFromAFE(std::array<uint16_t, CELLS>& cell_mV)
{
    // Use MCU ADC reference (e.g. 3.3 V), not AFE VREF (3.0 V). AFE gain 0.6 from REF_SEL=1.
    constexpr float ADC_FULL_SCALE = 4095.0f;
    constexpr float ADC_VREF_V     = 3.3f;   // STM32 VREF+; match your hardware
    constexpr float VCOUT_GAIN     = 0.6f;

    for (uint8_t cell = 0; cell < CELLS; ++cell)
    {
        if (bq.setCellForVCOUT(cell) != HAL_OK)
        {
            return HAL_ERROR;
        }

        // Datasheet t_VCOUT: allow ~100 µs+ for VCOUT to settle after mux change.
        HAL_Delay(10);

        // Blocking single conversion on VCOUT channel (configured as sole regular channel).
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            return HAL_ERROR;
        }
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            return HAL_ERROR;
        }

        uint16_t vcout_counts = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
        HAL_ADC_Stop(&hadc1);

        if (HAL_ADC_Start(&hadc1) != HAL_OK)
		{
			return HAL_ERROR;
		}
		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
		{
			HAL_ADC_Stop(&hadc1);
			return HAL_ERROR;
		}

		vcout_counts = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
		HAL_ADC_Stop(&hadc1);

        float v_vcout = (static_cast<float>(vcout_counts) / ADC_FULL_SCALE) * ADC_VREF_V;
        float v_cell  = v_vcout / VCOUT_GAIN;

        cell_mV[cell] = static_cast<uint16_t>(v_cell * 1000.0f + 0.5f);
    }

    return HAL_OK;
}
#endif /* !USE_STANDIN_SENSORS */

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

#if USE_STANDIN_SENSORS
		cellVoltages = kStandinCell_mV;
		bms.set_cell_mV(cellVoltages);
		cellTempADC = kStandinNtcCounts;
		bms.set_ntc_counts(cellTempADC);
#else
		// Production path: VCOUT + ADC + DataValidator.
		if (readCellsFromAFE(cellVoltages) != HAL_OK)
		{
			faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
		}
		else
		{
			faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			if (dataValidator.validateCellVoltages(cellVoltages) == 0)
			{
				bms.set_cell_mV(cellVoltages);
				faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
			}
			else
			{
				faultManager.setFault(FaultManager::FaultType::BQ76920_RESULT_ERROR);
			}
		}
#endif
		osMutexRelease(bmsMutex_id);

#if USE_STANDIN_SENSORS
		faultManager.clearFault(FaultManager::FaultType::ADC_RESULT_ERROR);
#else
		// Read temperatures from ADC DMA
		TempDMAComplete = false;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc_buf, ADC_CHANNEL_COUNT);
		while (!TempDMAComplete) {}

		for (int i = 0; i < CELLS-1; i++){
			cellTempADC[i] = adc_buf[i];
		}
        cellTempADC[5] = adc_buf[6];

		if(dataValidator.validateADCReadings(cellTempADC) == 0){
			osMutexAcquire(bmsMutex_id, osWaitForever);
			bms.set_ntc_counts(cellTempADC);
			osMutexRelease(bmsMutex_id);
			faultManager.clearFault(FaultManager::FaultType::ADC_RESULT_ERROR);
		}else{
			faultManager.setFault(FaultManager::FaultType::ADC_RESULT_ERROR);
		}
#endif

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


