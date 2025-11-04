/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
# define ADC_NUM_CONVERSIONS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t voltageMonitoringHandle;
const osThreadAttr_t voltageMonitoring_attributes = {
  .name = "voltageTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t temperatureMonitoringHandle;
const osThreadAttr_t temperatureMonitoring_attributes = {
  .name = "temperatureTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osMutexId_t bmsMutex_id;

const osMutexAttr_t BMS_Mutex_attr = {
  "BMSMutex",     // human readable mutex name
  osMutexRecursive,    // attr_bits
  NULL,                // memory for control block
  0U                   // size for control block
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
void StartDefaultTask(void *argument);
void StartVoltageTask(void *argument);
void StartTemperatureTask(void *argument);

/* USER CODE BEGIN PFP */
static int count = 0;
HAL_StatusTypeDef allCallback(const CANDriver::CANFrame& msg, void* ctx){
	count++;
	return HAL_OK;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BQ7692000PW bq(&hi2c2);
BMS bms(DeviceConfig::CELL_COUNT_CONF); // 4-cell configuration
extern CAN_HandleTypeDef hcan1;
static CANDriver::CANDevice can(&hcan1);
//static CanBus can(hcan1);

FaultManager faultManager;

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  bmsMutex_id = osMutexNew(&BMS_Mutex_attr);

  can.addCallbackAll(allCallback);
  can.StartCANDevice();

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  voltageMonitoringHandle = osThreadNew(StartVoltageTask, NULL, &voltageMonitoring_attributes);
  temperatureMonitoringHandle = osThreadNew(StartTemperatureTask, NULL, &temperatureMonitoring_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|NWC_Pin|OK_Pin
                          |ERROR_Pin|Fault_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TS1_GPIO_Port, TS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 NWC_Pin OK_Pin
                           ERROR_Pin Fault_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|NWC_Pin|OK_Pin
                          |ERROR_Pin|Fault_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TS1_Pin */
  GPIO_InitStruct.Pin = TS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TS1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	CANDriver::CANFrame avg_msg(0x101, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

	HAL_GPIO_WritePin(TS1_GPIO_Port, TS1_Pin, GPIO_PIN_SET);
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

	TempDMAComplete = false;
	  /* Infinite loop */
	  for(;;)
	  {

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
	  }
  /* USER CODE END 5 */
}

void StartVoltageTask(void *argument)
{

	bool voltage_read_success = true;
	CANDriver::CANFrame highvolt_msg(0x101, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

	for (;;)
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
	}
}

void StartTemperatureTask(void *argument)
{

	bool temp_read_success = true;
	CANDriver::CANFrame hightemp_msg(0x101, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 8);

	for (;;)
	{

		//external temperatures
		TempDMAComplete = false;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc_buf, ADC_NUM_CONVERSIONS);
		while(TempDMAComplete == false){};

		for(uint8_t i = 0; i < 5; i++){
			cellTempADC[i] = adc_buf[i];

			if (adc_buf[i] == 0 || adc_buf[i] == 4095) {
				faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, true);
				temp_read_success = false;
			}
			else
			{
				faultManager.clearFault(FaultManager::FaultType::ADC_READ_ERROR);
			}
		}


		//board temperatures (TMP112)

		// Get die temperature
		osMutexAcquire(bmsMutex_id, osWaitForever);
		if (bq.getDieTemp(&dieTemp) != HAL_OK) {
			faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
			temp_read_success = false;
		}
		else
		{
			faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
		}

		if (temp_read_success)
		{
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
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
