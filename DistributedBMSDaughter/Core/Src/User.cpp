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
#define USE_STANDIN_SENSORS 0
#endif

#include "User.hpp"

//Lower Level Drivers
// #include "BQ7692000.hpp"
#include "BQ76907.hpp"

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
BQ76907 bq(&hi2c2);
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

/* Debugger / Live Expressions — add any symbol below in STM32CubeIDE Live Expressions. */
volatile uint16_t g_dbg_cell_mV[6]{};
volatile uint16_t g_dbg_ntc_adc[6]{};
volatile float g_dbg_ntc_C[6]{};
volatile uint16_t g_dbg_avg_cell_mV = 0;
volatile uint16_t g_dbg_high_cell_mV = 0;
volatile uint16_t g_dbg_low_cell_mV = 0;
volatile float g_dbg_avg_C = 0.0f;
volatile float g_dbg_high_C = 0.0f;
volatile float g_dbg_low_C = 0.0f;
volatile uint8_t g_dbg_high_cell_idx = 0;
volatile uint8_t g_dbg_high_temp_idx = 0;
volatile uint8_t g_dbg_low_temp_idx = 0;
volatile uint16_t g_dbg_can_id = 0;
volatile uint32_t g_dbg_fault_mask = 0;
volatile uint32_t g_dbg_cycle_count = 0;
volatile uint32_t g_dbg_can_tx_ok = 0;
volatile uint32_t g_dbg_can_esr = 0;
volatile uint8_t g_dbg_can_state = 0;

static void refreshDebugMonitor(const BMS::Results& r)
{
    for (uint8_t i = 0; i < CELLS; ++i)
    {
        g_dbg_cell_mV[i] = cellVoltages[i];
        g_dbg_ntc_adc[i] = cellTempADC[i];
        g_dbg_ntc_C[i] = r.ntc_C[i];
    }
    g_dbg_avg_cell_mV = r.avg_cell_mV;
    g_dbg_high_cell_mV = r.high_cell_mV;
    g_dbg_low_cell_mV = r.low_cell_mV;
    g_dbg_avg_C = r.avg_C;
    g_dbg_high_C = r.high_C;
    g_dbg_low_C = r.low_C;
    g_dbg_high_cell_idx = r.high_cell_phys_idx;
    g_dbg_high_temp_idx = r.high_temp_idx;
    g_dbg_low_temp_idx = r.low_temp_idx;
    g_dbg_fault_mask = faultManager.getFaultMask();
    ++g_dbg_cycle_count;
}

#if !USE_STANDIN_SENSORS
// Real BQ76907 direct-command reads (used when USE_STANDIN_SENSORS is 0).
// Logical pack cell order (0..5) maps to BQ cell channels 1..5 and 7 (skip 6).
static constexpr std::array<uint8_t, CELLS> kBqCellMap1Based = {{1u, 2u, 3u, 4u, 5u, 7u}};

static HAL_StatusTypeDef readCellsFromAFE(std::array<uint16_t, CELLS>& cell_mV)
{
    for (uint8_t cell = 0; cell < CELLS; ++cell)
    {
        uint16_t mv = 0;
        if (bq.readCellVoltage(kBqCellMap1Based[cell], mv) != HAL_OK)
        {
            return HAL_ERROR;
        }
        cell_mV[cell] = mv;
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
    g_dbg_can_id = DeviceConfig::CAN_ID;

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

		/* Always push latest counts; BMS treats 0/saturated as NaN per channel. */
		bool any_valid_adc = false;
		const auto& adc_cfg = dataValidator.getConfig();
		for (uint16_t count : cellTempADC) {
		    if (count >= adc_cfg.min_adc_value && count <= adc_cfg.max_adc_value) {
		        any_valid_adc = true;
		        break;
		    }
		}

		osMutexAcquire(bmsMutex_id, osWaitForever);
		bms.set_ntc_counts(cellTempADC);
		osMutexRelease(bmsMutex_id);

		if (any_valid_adc) {
			faultManager.clearFault(FaultManager::FaultType::ADC_RESULT_ERROR);
		} else {
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
        refreshDebugMonitor(r);
        osMutexRelease(bmsMutex_id);

        /* ---------------- CAN housekeeping ---------------- */
        can1.poll();   // Required for recovery / error handling
        if (can1.state() == CanBus::State::Healthy) {
            faultManager.clearFault(FaultManager::FaultType::CAN_BUS_OFF);
        } else {
            faultManager.setFault(FaultManager::FaultType::CAN_BUS_OFF);
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

        g_dbg_can_tx_ok = can1.txOk();
        g_dbg_can_esr = hcan1.Instance->ESR;
        g_dbg_can_state = static_cast<uint8_t>(can1.state());

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


