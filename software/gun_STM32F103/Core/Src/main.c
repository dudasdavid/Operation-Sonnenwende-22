/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64
#define LINE_BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskOne */
osThreadId_t TaskOneHandle;
uint32_t myTask02Buffer[ 128 ];
osStaticThreadDef_t myTask02ControlBlock;
const osThreadAttr_t TaskOne_attributes = {
  .name = "TaskOne",
  .cb_mem = &myTask02ControlBlock,
  .cb_size = sizeof(myTask02ControlBlock),
  .stack_mem = &myTask02Buffer[0],
  .stack_size = sizeof(myTask02Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
uint32_t myTask03Buffer[ 128 ];
osStaticThreadDef_t myTask03ControlBlock;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .cb_mem = &myTask03ControlBlock,
  .cb_size = sizeof(myTask03ControlBlock),
  .stack_mem = &myTask03Buffer[0],
  .stack_size = sizeof(myTask03Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
uint32_t myTask04Buffer[ 128 ];
osStaticThreadDef_t myTask04ControlBlock;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .cb_mem = &myTask04ControlBlock,
  .cb_size = sizeof(myTask04ControlBlock),
  .stack_mem = &myTask04Buffer[0],
  .stack_size = sizeof(myTask04Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t rxBuffer[RX_BUFFER_SIZE]; // ToF sensor comm
uint8_t txBuffer[TX_BUFFER_SIZE]; // ToF sensor comm

volatile uint16_t rxLen = 0; // ToF sensor comm

static volatile uint32_t gpio4_reset_counter = 0;
static volatile uint32_t gpio4_last_counter = 0;
static volatile uint32_t gpio4_timestamp = 0;
static volatile uint32_t gpio4_last_timestamp = 0;
static volatile uint32_t gpio5_counter = 0;
static volatile uint32_t gpio5_timestamp = 0;
static volatile uint32_t gpio5_last_timestamp = 0;
static volatile uint32_t button_debounce = 150;

static volatile uint32_t ADC_Buf[4];
static volatile uint32_t ADC_Values[4];

static volatile float battery_voltage = 0;

static volatile uint8_t firing_sequence_start = 0;
static volatile uint32_t trigger_start_timestamp = 0;

volatile bool trigger_on = false;
volatile bool gpio4_reset_confirmed = true;
volatile uint32_t speed_up_threshold = 2000;

static volatile uint32_t sequence_time = 0;
static volatile uint32_t distance = 9999;
static uint8_t ammo_counter = 12;
volatile bool no_mag_flag = false;
volatile bool was_no_mag_flag = false;
volatile bool shot_is_happening = false;

uint8_t rxCommBuffer[RX_BUFFER_SIZE];
uint8_t txCommBuffer[TX_BUFFER_SIZE];
uint8_t lineBuffer[LINE_BUFFER_SIZE];

volatile uint16_t rxCommLen = 0;
uint16_t lineBufferIndex = 0;

uint8_t shot_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTaskOne(void *argument);
void StartDisplayTask(void *argument);
void StartCommTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

  if(hadc->Instance==ADC1){
      ADC_Values[0] = ADC_Buf[0];
      ADC_Values[1] = ADC_Buf[1];
      ADC_Values[2] = ADC_Buf[2];
      ADC_Values[3] = ADC_Buf[3];
      HAL_ADC_Stop_DMA(&hadc1);
  }
}
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart3, rxBuffer, RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  HAL_UART_Receive_DMA(&huart1, rxCommBuffer, RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
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

  /* creation of TaskOne */
  TaskOneHandle = osThreadNew(StartTaskOne, NULL, &TaskOne_attributes);

  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  htim1.Init.Period = 3600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_MOT_A_GPIO_Port, TRIG_MOT_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_EXTI_4_Pin */
  GPIO_InitStruct.Pin = TRIG_EXTI_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TRIG_EXTI_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : END_SW_EXTI_5_Pin */
  GPIO_InitStruct.Pin = END_SW_EXTI_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(END_SW_EXTI_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_MOT_A_Pin */
  GPIO_InitStruct.Pin = TRIG_MOT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_MOT_A_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
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
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET){
			trigger_on = true;
			if (gpio4_reset_confirmed) {
				gpio4_reset_confirmed = false;
				gpio4_reset_counter = 0;
				gpio4_timestamp = HAL_GetTick();
			}
		}
		else {
			trigger_on = false;
		}
		break;
	case GPIO_PIN_5:
		//TIM1->CCR2 = 0*3600/100; //immediately stop trigger motor
		gpio5_timestamp = HAL_GetTick();
		if (gpio5_timestamp > (gpio5_last_timestamp + button_debounce)){
			gpio5_last_timestamp = gpio5_timestamp;
			gpio5_counter++;
		}
		break;
	}
}

void I2C_Reset_Bus(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Deinit I2C peripheral
    HAL_I2C_DeInit(hi2c);

    // 2. Reconfigure PB8/PB9 as GPIO open-drain
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Manually clock SCL (PB8) if SDA (PB9) is stuck low
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    // 4. Generate a STOP condition: SDA HIGH while SCL HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // SDA low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL high
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // SDA high
    HAL_Delay(1);

    // 5. Re-init I2C peripheral
    MX_I2C1_Init(); // or HAL_I2C_Init(hi2c);
}

const uint16_t font_big_TL[] = {
    0x00FF, 0xFC00, 0x00FF, 0xFC00, 0x03FF, 0xFF00, 0x03FF, 0xFF00,
    0x0FFC, 0xFF00, 0x0FFC, 0xFF00, 0x3FF0, 0x3F00, 0x3FF0, 0x3F00,
    0x3FC0, 0x0F00, 0x3FC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00,
    0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFF00, 0x0300, 0xFF00, 0x0300,
    0xFF00, 0x0300, 0xFF00, 0x0300, 0xFF00, 0x0300, 0xFF00, 0x0300,
    0xFF00, 0x0300, 0xFF00, 0x0300, 0xFF00, 0x0300, 0xFF00, 0x0300,
    0xFF00, 0x0300, 0xFF00, 0x0300, 0x00FF, 0x0000, 0x00FF, 0x0000,
    0x00FF, 0xFC00, 0x00FF, 0xFC00, 0x00FF, 0xFF00, 0x00FF, 0xFF00,
    0x00FF, 0xFF00, 0x00FF, 0xFF00, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x003F, 0xFF00, 0x003F, 0xFF00, 0x03FF, 0xFF00, 0x03FF, 0xFF00,
    0x0FFC, 0x0F00, 0x0FFC, 0x0F00, 0x0FF0, 0x0000, 0x0FF0, 0x0000,
    0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000,
    0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x0FF0, 0x0000, 0x0FF0, 0x0000,
    0x0FF0, 0x0000, 0x0FF0, 0x0000, 0x0FFC, 0x0000, 0x0FFC, 0x0000,
    0x03FF, 0x0000, 0x03FF, 0x0000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x003F, 0xF000, 0x003F, 0xF000, 0x00FF, 0xFF00, 0x00FF, 0xFF00,
    0x03FF, 0xFF00, 0x03FF, 0xFF00, 0x0FFC, 0x0F00, 0x0FFC, 0x0F00,
    0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000,
    0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x0FF0, 0x0000, 0x0FF0, 0x0000,
    0x0FF0, 0x0000, 0x0FF0, 0x0000, 0x03FF, 0x0000, 0x03FF, 0x0000,
    0x00FF, 0xFF00, 0x00FF, 0xFF00, 0x03FF, 0xFF00, 0x03FF, 0xFF00,
    0x0FFC, 0x0000, 0x0FFC, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000,
    0x03FC, 0x0000, 0x03FC, 0x0000, 0x03FF, 0x0000, 0x03FF, 0x0000,
    0x03FF, 0x0000, 0x03FF, 0x0000, 0x03FF, 0xC000, 0x03FF, 0xC000,
    0x03FF, 0xF000, 0x03FF, 0xF000, 0x03FF, 0xFC00, 0x03FF, 0xFC00,
    0x03FF, 0xFC00, 0x03FF, 0xFC00, 0x03FC, 0xFF00, 0x03FC, 0xFF00,
    0x03FC, 0x3F00, 0x03FC, 0x3F00, 0x03FC, 0x3F00, 0x03FC, 0x3F00,
    0x03FC, 0x0F00, 0x03FC, 0x0F00, 0x03FC, 0x0300, 0x03FC, 0x0300,
    0x03FC, 0x0300, 0x03FC, 0x0300, 0x0FFF, 0xFF00, 0x0FFF, 0xFF00,
    0x0FFF, 0xFF00, 0x0FFF, 0xFF00, 0x0FFF, 0xFF00, 0x0FFF, 0xFF00,
    0x0000, 0x3F00, 0x0000, 0x3F00, 0x0000, 0x3F00, 0x0000, 0x3F00,
    0x0000, 0x3F00, 0x0000, 0x3F00, 0x0000, 0x3F00, 0x0000, 0x3F00,
    0x0000, 0x3F00, 0x0000, 0x3F00, 0x003F, 0xFF00, 0x003F, 0xFF00,
    0x03FF, 0xFF00, 0x03FF, 0xFF00, 0x0FFF, 0x0000, 0x0FFF, 0x0000,
    0x0FFC, 0x0000, 0x0FFC, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000,
    0x0FFF, 0xC000, 0x0FFF, 0xC000, 0x3FFF, 0xFC00, 0x3FFF, 0xFC00,
    0x3F03, 0xFF00, 0x3F03, 0xFF00, 0x0000, 0xFF00, 0x0000, 0xFF00,
    0x0000, 0x3F00, 0x0000, 0x3F00, 0x0000, 0x3F00, 0x0000, 0x3F00,
    0x0000, 0x0F00, 0x0000, 0x0F00, 0x0000, 0x0F00, 0x0000, 0x0F00,
    0x03FF, 0xCF00, 0x03FF, 0xCF00, 0x0FFF, 0xFF00, 0x0FFF, 0xFF00,
    0x3FF0, 0xFF00, 0x3FF0, 0xFF00, 0xFFC0, 0x3F00, 0xFFC0, 0x3F00,
    0xFF00, 0x0F00, 0xFF00, 0x0F00, 0xFFFF, 0xFF00, 0xFFFF, 0xFF00,
    0xFFFF, 0xFF00, 0xFFFF, 0xFF00, 0xFFFF, 0xFF00, 0xFFFF, 0xFF00,
    0xFF00, 0x0000, 0xFF00, 0x0000, 0x3FC0, 0x0000, 0x3FC0, 0x0000,
    0x3FC0, 0x0000, 0x3FC0, 0x0000, 0x0FF0, 0x0000, 0x0FF0, 0x0000,
    0x03F0, 0x0000, 0x03F0, 0x0000, 0x03FC, 0x0000, 0x03FC, 0x0000,
    0x00FF, 0x0000, 0x00FF, 0x0000, 0x00FF, 0x0000, 0x00FF, 0x0000,
    0x003F, 0xC000, 0x003F, 0xC000, 0x003F, 0xC000, 0x003F, 0xC000,
    0x03FF, 0xFC00, 0x03FF, 0xFC00, 0x0FFF, 0xFF00, 0x0FFF, 0xFF00,
    0x3FF0, 0xFF00, 0x3FF0, 0xFF00, 0x3FC0, 0x3F00, 0x3FC0, 0x3F00,
    0x3FC0, 0x3F00, 0x3FC0, 0x3F00, 0x3FC0, 0x3F00, 0x3FC0, 0x3F00,
    0x3FC0, 0x3F00, 0x3FC0, 0x3F00, 0x0FF0, 0xFF00, 0x0FF0, 0xFF00,
    0x03FF, 0xFF00, 0x03FF, 0xFF00, 0x00FF, 0xFC00, 0x00FF, 0xFC00,
    0x03FF, 0xFF00, 0x03FF, 0xFF00, 0x0FFF, 0x3F00, 0x0FFF, 0x3F00,
    0x3FF0, 0x3F00, 0x3FF0, 0x3F00, 0x00FF, 0xFC00, 0x00FF, 0xFC00,
    0x03FF, 0xFF00, 0x03FF, 0xFF00, 0x0FFC, 0x3F00, 0x0FFC, 0x3F00,
    0x3FF0, 0x0F00, 0x3FF0, 0x0F00, 0x3FC0, 0x0F00, 0x3FC0, 0x0F00,
    0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00,
    0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00,
    0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFF0, 0x3F00, 0xFFF0, 0x3F00,
    0xFFFF, 0xFF00, 0xFFFF, 0xFF00, 0xFF3F, 0xFC00, 0xFF3F, 0xFC00,
};

const uint16_t font_big_TR[] = {
    0xFF00, 0x0300, 0xFF00, 0x0300, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00,
    0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0x3FC0, 0x0F00, 0x3FC0, 0x0F00,
    0x3FF0, 0x3F00, 0x3FF0, 0x3F00, 0x0FFC, 0xFF00, 0x0FFC, 0xFF00,
    0x03FF, 0xFF00, 0x03FF, 0xFF00, 0x00FF, 0xFC00, 0x00FF, 0xFC00,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000, 0x00FF, 0xC000,
    0x00FF, 0xC000, 0x00FF, 0xC000, 0xFFFF, 0xFF00, 0xFFFF, 0xFF00,
    0xFFFF, 0xFF00, 0xFFFF, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x000F, 0xFC00, 0x000F, 0xFC00, 0x0003, 0xFC00, 0x0003, 0xFC00,
    0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x3F00, 0x0000, 0x3F00,
    0x0000, 0x3F00, 0x0000, 0x3F00, 0x0000, 0x0F00, 0x0000, 0x0F00,
    0x3FFF, 0xFF00, 0x3FFF, 0xFF00, 0x3FFF, 0xFF00, 0x3FFF, 0xFF00,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x3FC0, 0x0000, 0x3FC0, 0x0000,
    0x3FC0, 0x0000, 0x3FC0, 0x0000, 0x3FC0, 0x0000, 0x3FC0, 0x0000,
    0x3FC0, 0x0000, 0x3FC0, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000,
    0x0FFC, 0x0F00, 0x0FFC, 0x0F00, 0x03FF, 0xFF00, 0x03FF, 0xFF00,
    0x003F, 0xFF00, 0x003F, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xFFFF, 0xFF00, 0xFFFF, 0xFF00, 0xFFFF, 0xFF00, 0xFFFF, 0xFF00,
    0x03FC, 0x0000, 0x03FC, 0x0000, 0x03FC, 0x0000, 0x03FC, 0x0000,
    0x03FC, 0x0000, 0x03FC, 0x0000, 0x03FC, 0x0000, 0x03FC, 0x0000,
    0x03FC, 0x0000, 0x03FC, 0x0000, 0x03FC, 0x0000, 0x03FC, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000,
    0x3FC0, 0x0000, 0x3FC0, 0x0000, 0x3FF0, 0x0000, 0x3FF0, 0x0000,
    0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x0FF0, 0x0000, 0x0FF0, 0x0000,
    0x0FFC, 0x0F00, 0x0FFC, 0x0F00, 0x03FF, 0xFF00, 0x03FF, 0xFF00,
    0x003F, 0xFF00, 0x003F, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xFF00, 0x0F00, 0xFF00, 0x0F00, 0xFF00, 0x0F00, 0xFF00, 0x0F00,
    0xFF00, 0x0F00, 0xFF00, 0x0F00, 0xFF00, 0x3F00, 0xFF00, 0x3F00,
    0xFFC0, 0x3F00, 0xFFC0, 0x3F00, 0x3FF0, 0xFF00, 0x3FF0, 0xFF00,
    0x0FFF, 0xFF00, 0x0FFF, 0xFF00, 0x00FF, 0xF000, 0x00FF, 0xF000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x000F, 0xF000, 0x000F, 0xF000,
    0x000F, 0xF000, 0x000F, 0xF000, 0x0003, 0xFC00, 0x0003, 0xFC00,
    0x0003, 0xFF00, 0x0003, 0xFF00, 0x0003, 0xFF00, 0x0003, 0xFF00,
    0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0xFF00,
    0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0xFFC0, 0x0F00,
    0xFF00, 0x0F00, 0xFF00, 0x0F00, 0xFF00, 0x0F00, 0xFF00, 0x0F00,
    0xFFC0, 0x0F00, 0xFFC0, 0x0F00, 0x3FF0, 0xFF00, 0x3FF0, 0xFF00,
    0x0FFF, 0xFF00, 0x0FFF, 0xFF00, 0x00FF, 0xFC00, 0x00FF, 0xFC00,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xFFC0, 0x0000, 0xFFC0, 0x0000,
    0x3FC0, 0x0000, 0x3FC0, 0x0000, 0x3FC0, 0x0000, 0x3FC0, 0x0000,
    0x3FF0, 0x0000, 0x3FF0, 0x0000, 0x0FF0, 0x0000, 0x0FF0, 0x0000,
    0x03FF, 0x0300, 0x03FF, 0x0300, 0x00FF, 0xFF00, 0x00FF, 0xFF00,
    0x003F, 0xFF00, 0x003F, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};

const uint16_t font_big_BL[] = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};

const uint16_t font_big_BR[] = {
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xFF00, 0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xC000, 0x0000, 0xC000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000, 0xFC00, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000, 0xF000, 0x0000,
    0xC000, 0x0000, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};


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

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);

  //Initialize trigger motor controls
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);


  //Initialize speeder motor controls
  TIM1->CCR1 = 100*3600/100;
  TIM1->CCR2 = 100*3600/100;

  TIM1->CCR3 = 100*3600/100;
  TIM1->CCR4 = 100*3600/100;


  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    if (trigger_on == false) {
    	gpio4_reset_counter++;
    	if (gpio4_reset_counter > 3){
    		gpio4_reset_confirmed = true;
    	}
    }
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    battery_voltage = ADC_Values[0] / 410.0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buf,4);
	if ( sequence_time > speed_up_threshold) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}


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
						ammo_counter = 0;
						shot_counter = 0;
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



  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskOne */
/**
* @brief Function implementing the TaskOne thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOne */
void StartTaskOne(void *argument)
{
  /* USER CODE BEGIN StartTaskOne */
  /* Infinite loop */
  for(;;)
  {

	if ((trigger_on == true || shot_counter > 0) && ammo_counter > 0){

		shot_is_happening = true;
		sequence_time = HAL_GetTick() - gpio4_timestamp;

		// note: sequence starts from else branch and goes up on the tree!
		if ( sequence_time > 1000) {
			TIM1->CCR1 = 95*3600/100;
			TIM1->CCR2 = 0*3600/100;
		}
		else if ( sequence_time > 800) {
			TIM1->CCR1 = 85*3600/100;
			TIM1->CCR2 = 0*3600/100;
		}
		else if ( sequence_time > 600) {
			TIM1->CCR1 = 70*3600/100;
			TIM1->CCR2 = 0*3600/100;
		}
		else if ( sequence_time > 400) {
			// then give it a kick to overcome static friction of the spinner wheels
			TIM1->CCR1 = 50*3600/100;
			TIM1->CCR2 = 0*3600/100;
		}
		else {
			// in the first few 100 ms give time to the motor to go from dynamic braking to high Z
			TIM1->CCR1 = 0*3600/100;
			TIM1->CCR2 = 0*3600/100;
		}

		// volatile uint32_t speed_up_threshold = 3500;
		if ( sequence_time > speed_up_threshold) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			TIM1->CCR4 = 0*3600/100;
			TIM1->CCR3 = 100*3600/100;
			osDelay(500);
			ammo_counter--;
			shot_counter--;

			if (ammo_counter == 0) {
				shot_counter = 0;
			}

			if (ammo_counter == 0 || (shot_counter == 0 && gpio4_reset_confirmed == true)) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				// o to dynamic breaking
				TIM1->CCR1 = 100*3600/100;
				TIM1->CCR2 = 100*3600/100;
			}

			TIM1->CCR4 = 80*3600/100;
			TIM1->CCR3 = 0*3600/100;
			osDelay(300);
			TIM1->CCR3 = 100*3600/100;
			TIM1->CCR4 = 100*3600/100;
			osDelay(500);
			shot_is_happening = false;
		}


	}
	else {
		sequence_time = 0;
		if (ammo_counter == 0 || (shot_counter == 0 && gpio4_reset_confirmed == true)) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			// go to dynamic breaking
			TIM1->CCR1 = 100*3600/100;
			TIM1->CCR2 = 100*3600/100;
		}
	}

    osDelay(20);
  }
  /* USER CODE END StartTaskOne */
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

	// resolution is 128x64
	  ssd1306_Init();

	  char str[16];
	  int int_part = 0;
	  int frac_part = 0;

	  ssd1306_Fill(Black);
	  ssd1306_SetCursor(20, 30);
	  ssd1306_WriteString("Welcome!", Font_11x18, White);
	  ssd1306_UpdateScreen();

	  osDelay(3000);
	  ssd1306_Fill(Black);

	  int blinking_counter = 0;

  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    sprintf(str, "%lu mm", distance);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 56);
    ssd1306_WriteString(str, Font_6x8, White);


    int_part = (int)battery_voltage;                      // 3
    frac_part = (int)((battery_voltage - int_part) * 10); // 1

    if (frac_part < 0) frac_part = -frac_part; // handle negative decimals

    sprintf(str, "%d.%d V", int_part, frac_part);
    ssd1306_SetCursor(98, 56);
    ssd1306_WriteString(str, Font_6x8, White);

    if (no_mag_flag){
    	blinking_counter++;
    }
    else {
    	blinking_counter = 0;
    }
    if (blinking_counter < 10){
        sprintf(str, "%02d", ammo_counter);
        //sprintf(str, "%d %d", ammo_counter / 10, ammo_counter % 10); // not nice with a space
        ssd1306_SetCursor(7, 4);
        ssd1306_WriteBigChar('0' + (ammo_counter / 10), font_big_TL, font_big_TR, font_big_BL, font_big_BR, White, 270);
        ssd1306_SetCursor(54, 4);
        ssd1306_WriteBigChar('0' + (ammo_counter % 10), font_big_TL, font_big_TR, font_big_BL, font_big_BR, White, 270);
        //ssd1306_WriteString(str, Font_16x26, White);
    }
    if (blinking_counter > 15) {
    	blinking_counter = 0;
    }


    ssd1306_UpdateScreen();

  }
  /* USER CODE END StartDisplayTask */
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
		  else if (strncmp((char*)lineBuffer, "RSTD", 4) == 0)
		  {
			  I2C_Reset_Bus(&hi2c1);
			  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "RESET DISPLAY\r\n");
		  }
		  else if (strncmp((char*)lineBuffer, "SHOT:", 5) == 0)
		  {
              char ch = lineBuffer[5];

              // Check if it's a digit between '1' and '9'
              if (ch >= '1' && ch <= '9')
              {
                  shot_counter = ch - '0';

                  snprintf((char*)txCommBuffer, TX_BUFFER_SIZE, "SHOT:%d\r\n", shot_counter);

                  gpio4_timestamp = HAL_GetTick();
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

	  }
  /* USER CODE END StartCommTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
