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
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "GC9A01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE   64
#define TX_BUFFER_SIZE   64
#define LINE_BUFFER_SIZE 64
#define ADC_BUF_LEN      32 // must be even (half-buffer processing)
#define VREF             3.3f  // VDDA if you don’t compensate with Vrefint
#define DIV_SCALE        ((9.5f + 21.2f) / 9.5f)  // 3.2 for 10k/22k
#define ADC_MAX          4095.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hostRxTask */
osThreadId_t hostRxTaskHandle;
const osThreadAttr_t hostRxTask_attributes = {
  .name = "hostRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hostTxTask */
osThreadId_t hostTxTaskHandle;
const osThreadAttr_t hostTxTask_attributes = {
  .name = "hostTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t rxBuffer[RX_BUFFER_SIZE]; // ToF sensor comm
uint8_t txBuffer[TX_BUFFER_SIZE]; // ToF sensor comm

volatile uint16_t rxLen = 0; // ToF sensor comm

static volatile uint32_t adc_buf[ADC_BUF_LEN] __attribute__((aligned(4)));

static volatile float battery_voltage = 0;

volatile bool magazine = false;
volatile bool trigger1 = false;
volatile bool trigger2 = false;
volatile bool pusher_switch = false;
volatile bool trigger2_latched = false;
volatile bool need_false = false;
volatile bool armed = false;

static volatile uint32_t distance = 9999;
static uint8_t ammo_counter = 12;
volatile bool no_mag_flag = false;
volatile bool was_no_mag_flag = false;
volatile bool shot_is_happening = false;
uint8_t shot_counter = 0;

uint8_t rxCommBuffer[RX_BUFFER_SIZE];
uint8_t txCommBuffer[TX_BUFFER_SIZE];
uint8_t lineBuffer[LINE_BUFFER_SIZE];
volatile uint16_t rxCommLen = 0;
uint16_t lineBufferIndex = 0;

static volatile uint32_t shot_timestamp = 0;
volatile uint32_t speed_up_threshold = 2000;
volatile uint32_t shooting_cooldown = 1500;
static volatile uint32_t sequence_time = 0;
static volatile uint32_t last_shot = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartCommTask(void *argument);
void StartDisplayTask(void *argument);
void StartSensorTask(void *argument);
void StartHostRxTask(void *argument);
void StartHostTxTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
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

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of hostRxTask */
  hostRxTaskHandle = osThreadNew(StartHostRxTask, NULL, &hostRxTask_attributes);

  /* creation of hostTxTask */
  hostTxTaskHandle = osThreadNew(StartHostTxTask, NULL, &hostTxTask_attributes);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_DC_Pin|SPI1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_DC_Pin SPI1_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_DC_Pin|SPI1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_4:
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET){
			pusher_switch = true;
		}
		else {
			pusher_switch = false;
		}
		break;
	case GPIO_PIN_5:
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET){
			magazine = true;
		}
		else {
			magazine = false;
		}
		break;
	case GPIO_PIN_8:
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET){
			trigger1 = true;
		}
		else {
			trigger1 = false;
		}
		break;
	case GPIO_PIN_9:
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET){
			trigger2 = true;
		}
		else {
			trigger2 = false;
		}
		break;
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) {
        process_half(0, ADC_BUF_LEN / 2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) {
        process_half(ADC_BUF_LEN / 2, ADC_BUF_LEN / 2);
    }
}

static void process_half(size_t offset, size_t count)
{
    uint32_t sum = 0;

    // If your buffer is uint32_t, the 12-bit value sits in the lower bits (right-aligned)
    for (size_t i = 0; i < count; ++i) {
        sum += (uint16_t)adc_buf[offset + i];
    }

    uint16_t avg_raw = (uint16_t)(sum / count);

    float v_pin  = (avg_raw * VREF) / ADC_MAX;
    battery_voltage = v_pin * DIV_SCALE;
}
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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);

  //Initialize speeder motor controls
  TIM1->CCR1 = 0*5000/100;
  TIM1->CCR2 = 0*5000/100;

  TIM1->CCR3 = 0*5000/100;
  TIM1->CCR4 = 0*5000/100;

  uint16_t i = 0;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  /* Infinite loop */
  for(;;)
  {


	if (trigger1 && trigger2) {
		shot_counter = 1;
	}

	// automatic control
	if (shot_counter > 0 && ammo_counter > 0){
		shot_is_happening = true;
		sequence_time = HAL_GetTick() - shot_timestamp;

		// note: sequence starts from else branch and goes up on the tree!
		if ( sequence_time > 700) {
			TIM1->CCR1 = 90*5000/100;
			TIM1->CCR2 = 0*5000/100;
		}
		else if ( sequence_time > 500) {
			TIM1->CCR1 = 87*5000/100;
			TIM1->CCR2 = 0*5000/100;
		}
		else if ( sequence_time > 400) {
			TIM1->CCR1 = 85*5000/100;
			TIM1->CCR2 = 0*5000/100;
		}
		else if ( sequence_time > 200) {
			// then give it a kick to overcome static friction of the spinner wheels
			TIM1->CCR1 = 80*5000/100;
			TIM1->CCR2 = 0*5000/100;
		}
		else {
			// in the first few 100 ms give time to the motor to go from dynamic braking to high Z
			TIM1->CCR1 = 0*5000/100;
			TIM1->CCR2 = 0*5000/100;
		}

		// volatile uint32_t speed_up_threshold = 3500;
		if ( sequence_time > speed_up_threshold) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			if (HAL_GetTick() - last_shot > shooting_cooldown) {
				TIM1->CCR3 = 70*5000/100;
				TIM1->CCR4 = 0*5000/100;
			}
			else {
				TIM1->CCR3 = 0*5000/100;
				TIM1->CCR4 = 0*5000/100;
			}

			if (pusher_switch == false) {
				armed = true;
			}

			if (armed && pusher_switch) {
				ammo_counter--;
				shot_counter--;
				armed=false;
				last_shot = HAL_GetTick();
			}


			if (ammo_counter == 0) {
				shot_counter = 0;
			}

			if (ammo_counter == 0 || shot_counter == 0) {
				// go to dynamic breaking
				TIM1->CCR1 = 100*5000/100;
				TIM1->CCR2 = 100*5000/100;
				shot_is_happening = false;
			}

		}

	}
	else {
		TIM1->CCR1 = 100*5000/100;
		TIM1->CCR2 = 100*5000/100;
		TIM1->CCR3 = 0*5000/100;
		TIM1->CCR4 = 0*5000/100;
		shot_is_happening = false;
	}

    osDelay(1);

    i++;
    if ((i == 300) || (i == 500) || (i == 800) || (i == 1000)){
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    if (i==1500) {
	    i = 0;
    }


  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  /* Infinite loop */
  for(;;)
  {

	// Wait for notification from ISR
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	// Process received data
	if(rxLen > 0)
	{
		// Look for '\n'
		char* newlinePos = memchr(rxBuffer, '\n', rxLen);
		if(newlinePos)
		{
			size_t msgLen = newlinePos - (char*)rxBuffer;
			if(msgLen >= RX_BUFFER_SIZE) msgLen = RX_BUFFER_SIZE - 1;

			// Null-terminate
			rxBuffer[msgLen] = '\0';

			// Look for "mm" in the string
			char *mmPtr = strstr((char*)rxBuffer, "mm");
			if (mmPtr != NULL) {
				*mmPtr = '\0'; // cut off at "mm"
				distance = atoi((char*)rxBuffer); // convert number part

				if (shot_is_happening == false){

					if (distance > 20) {
						//ammo_counter = 0;
						//shot_counter = 0;
					}
					else {
						if (was_no_mag_flag){
							ammo_counter = 12;
							was_no_mag_flag = false;
						}
					}

					if (distance > 70) {
						no_mag_flag = true;
						was_no_mag_flag = true;
					}
					else {
						no_mag_flag = false;
					}

				}


			} else {
				distance = 9999;
			}
		}
	}

	// Reset buffer length
	rxLen = 0;

    osDelay(1);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	GC9A01_Init();

	GC9A01_FillRect(50,50,50,50,0xFFFF);

  /* Infinite loop */
  for(;;)
  {
      //gc9a01_fill_screen(GC9A01_BLUE);
      //gc9a01_draw_hline(10, 10, 220, GC9A01_WHITE);
      //gc9a01_draw_vline(10, 10, 220, GC9A01_WHITE);
      //gc9a01_draw_pixel(120, 120, GC9A01_RED);

      // Draw a gradient block using streaming API
      /*
      uint16_t w = 100, h = 100; uint16_t x=70, y=70;
      gc9a01_start_write_window(x,y,w,h);
      for (uint32_t j=0;j<h;j++) {
          for (uint32_t i=0;i<w;i++) {
              uint8_t r = (i*255)/w;
              uint8_t g = (j*255)/h;
              uint8_t b = 128;
              uint16_t c = GC9A01_COLOR(r,g,b);
              gc9a01_write_color_repeat(c, 1);
          }
      }
      */
      //gc9a01_end_write();
    osDelay(1000);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
  /* Infinite loop */
  for(;;)
  {

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET){
		pusher_switch = true;
	}
	else {
		pusher_switch = false;
	}

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET){
		magazine = true;
	}
	else {
		magazine = false;
	}

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET){
		trigger1 = true;
	}
	else {
		trigger1 = false;
	}

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET){
		trigger2 = true;
		if (need_false) {
			trigger2_latched = true;
		}

	}
	else {
		trigger2 = false;
		need_false = true;
	}

	osDelay(10);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartHostRxTask */
/**
* @brief Function implementing the hostRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHostRxTask */
void StartHostRxTask(void *argument)
{
  /* USER CODE BEGIN StartHostRxTask */
  HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* Infinite loop */
  for(;;)
  {
      // Wait for notification from ISR
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      // rxCommBuffer[0..rxCommLen] ends with \n, safe to process
      //memcpy(lineBuffer, rxCommBuffer, rxCommLen);
      lineBuffer[lineBufferIndex] = '\0'; // Null-terminate
      lineBufferIndex = 0;


	  // Build response
	  // Check for prefix "PWD:"
	  if (strncmp((char*)lineBuffer, "AMMO", 4) == 0)
	  {
		  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "AMMO:%d\r\n", ammo_counter);
	  }
	  else if (strncmp((char*)lineBuffer, "DIST", 4) == 0)
	  {
		  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "DIST:%d\r\n", (int)distance);
	  }
	  else if (strncmp((char*)lineBuffer, "MAG", 3) == 0)
	  {
		  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "MAG:%s\r\n", no_mag_flag ? "False" : "True");
	  }
	  else if (strncmp((char*)lineBuffer, "BAT", 3) == 0)
	  {
		  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "BAT:%d\r\n", (int)(battery_voltage*10));
	  }
	  else if (strncmp((char*)lineBuffer, "STAT", 4) == 0)
	  {
		  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "STAT:%d;%d;%d;%s;END\r\n", ammo_counter, (int)distance, (int)(battery_voltage*10), no_mag_flag ? "False" : "True");
	  }
	  else if (strncmp((char*)lineBuffer, "SHOT:", 5) == 0)
	  {
          char ch = lineBuffer[5];

          // Check if it's a digit between '1' and '9'
          if (ch >= '1' && ch <= '9')
          {
              shot_counter = ch - '0';

              snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "SHOT:%d\r\n", shot_counter);

              shot_timestamp = HAL_GetTick();
          }
          else
          {
        	  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "SHOT ERROR:%.50s\r\n", lineBuffer);
          }

	  }
	  else
	  {
		  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "ERR:%.50s\r\n", lineBuffer);
	  }

	  // Send response
	  HAL_UART_Transmit(&huart1, txCommBuffer, strlen((char*)txCommBuffer), HAL_MAX_DELAY);
	  osDelay(1);
  }
  /* USER CODE END StartHostRxTask */
}

/* USER CODE BEGIN Header_StartHostTxTask */
/**
* @brief Function implementing the hostTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHostTxTask */
void StartHostTxTask(void *argument)
{
  /* USER CODE BEGIN StartHostTxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHostTxTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
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
