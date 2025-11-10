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

#include "CanDriver.hpp"
#include "BmsFleet.hpp"
#include "UartFleetPack.hpp"

#include <stdint.h>
#include <stddef.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SOF0 0xA5
#define SOF1 0x5A


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

extern CAN_HandleTypeDef hcan1;
//static CanBus can(hcan1);
static CANDriver::CANDevice can(&hcan1);

static BmsFleet fleet;

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/**
 * Encode a payload into a framed UART packet.
 *
 * @param payload      Pointer to payload bytes to send.
 * @param payload_len  Number of payload bytes.
 * @param out_buf      Destination buffer for encoded frame.
 * @param out_max      Capacity of out_buf in bytes.
 * @return             Encoded frame length on success; 0 on error (e.g., not enough space).
 */


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

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_USART2_UART_Init();


	/* USER CODE BEGIN 2 */

	osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
	can.AddFilterRange(0x101, 4, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 0);
	can.addCallbackRange(0x101, 4, SG_CAN_ID_STD, daughterOneCallback, NULL);
	can.AddFilterId(0x101, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 0);
	can.addCallbackId(0x101, SG_CAN_ID_STD, daughterOneCallback, NULL);

	// Add Daughters 2-6 here

	can.addCallbackAll(allCallback);
	fleet.register_node(0x101, 0);


	can.StartCANDevice();
  /* Create the thread(s) */
  /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
    osKernelStart();



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

//		CanBus::Frame rx;
//		if (can.read(rx)) {
//			fleet.handle(rx, HAL_GetTick());
//			HAL_GPIO_TogglePin(OK_GPIO_Port, OK_Pin);
//			can.sendStd(0x21, {0x67,67}, 2);
//		}



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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, ERROR_Pin|OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ERROR_Pin OK_Pin */
  GPIO_InitStruct.Pin = ERROR_Pin|OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
static volatile uint32_t g_lastHeartbeatInterval = 0;

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	uint8_t txbuf[64];  // Buffer for encoded frame (SOF + LEN + payload + CRC)
	static uint8_t module_cursor = 0;
	static uint32_t last_heartbeat_ms = 0;
	static uint32_t heartbeat_counter = 0;
	static uint8_t frame_rotation = 0;  // 0=fleet, 1=module, 2=heartbeat
	uint32_t now = 0;

	for(;;)
	{
		now = osKernelGetTickCount();

		if (last_heartbeat_ms == 0U) {
			last_heartbeat_ms = now;
		}

		bool have_data = false;
		for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
			if (fleet.has_any_data(i)) {
				have_data = true;
				break;
			}
		}

		// Rotate between frame types to avoid overwhelming the receiver
		switch (frame_rotation) {
		case 0:  // Fleet summary
			if (have_data) {
				size_t txlen = uart_make_fleet_summary(fleet, now, txbuf, sizeof(txbuf));
				if (txlen > 0) {
					HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, txbuf, txlen, 1000);
					if (status != HAL_OK) {
						// TODO: handle transmission error (set fault flag?)
					} else {
						// Give receiver time to process frame
						osDelay(50);
					}
				}
			}
			frame_rotation = 1;
			break;

		case 1:  // Module summary
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
			frame_rotation = 2;
			break;

		case 2:  // Heartbeat
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

		osDelay(250);
	}
  /* USER CODE END 5 */
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
