/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
int cnt,cnt_old ;
int tim4Cnt,tim5Cnt,tim2Cnt;
int feedback;
int laps=0;
uint8_t Xpos,Xneg,Ypos,Yneg,Zpos,Zneg;

extern volatile int32_t xPul;
extern volatile int32_t yPul;
extern uint8_t xSpeed, ySpeed;
extern uint8_t xSen;
extern uint8_t ySen;
int highMode = 0;
int8_t rxbuf[32];
int spin=0;
int solder_push=0;
int push_length=0;
int spin_length=0;
int grating_scale_1=0;
int grating_scale_2=0;
int PGsend;
size_t k = 99;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

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
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */



  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  TIM5->CNT=765432;
  TIM2->CNT=998998;

  //initiate USB
  MX_USB_DEVICE_Init();


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

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

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
// LED light:
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
//
//	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);






    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_Delay(10);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 33888;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 33888;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XCLK_GPIO_Port, XCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YCLK_GPIO_Port, YCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : XCLK_Pin */
  GPIO_InitStruct.Pin = XCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(XCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_UP_Pin */
  GPIO_InitStruct.Pin = JOY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(JOY_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XDIR_Pin */
  GPIO_InitStruct.Pin = XDIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(XDIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : YDIR_Pin */
  GPIO_InitStruct.Pin = YDIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(YDIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Z__Pin Z_E11_Pin Y__Pin Y_E13_Pin */
  GPIO_InitStruct.Pin = Z__Pin|Z_E11_Pin|Y__Pin|Y_E13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : X__Pin */
  GPIO_InitStruct.Pin = X__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(X__GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : X_E15_Pin */
  GPIO_InitStruct.Pin = X_E15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(X_E15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : YCLK_Pin */
  GPIO_InitStruct.Pin = YCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(YCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_PB7);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
		{

		}



	if(htim->Instance == htim2.Instance){

		 cnt=__HAL_TIM_GET_COUNTER(&htim5)-750000;

//		   if (	(	(cnt)<=(htim5.Init.Period)*0.5	)		&&
//		 		( 	(cnt)>=0 	)			&&
//		 		(	cnt_old<=(htim5.Init.Period)	)			&&
//		 		(	cnt_old>=(htim5.Init.Period)*0.5	)
//		 		  	  )
//		   {
//
//		 	  laps++;
//		   }
//		   else  if 	(	(	(cnt)<=	(htim5.Init.Period))		&&
//		 			( 	(cnt)>=	(htim5.Init.Period)*0.5	)			&&
//		 			(	cnt_old<=	(htim5.Init.Period)*0.5	)			&&
//		 			(	cnt_old>=0	)
//		 			  	  )
//		   {
//
//		 	  laps--;
//		   }
//
//		   cnt_old=TIM5->CNT;

			 			 			feedback=(laps*htim5.Init.Period)+cnt;
	}

}
//  if (	(	(TIM5->CNT)<=(5)	)		&&
//		( 	(TIM5->CNT)>=0 	)			&&
//		(	cnt_old<=(10)	)			&&
//		(	cnt_old>=(5)	)
//		  	  )
//  {
//
//	  laps++;
//  }
//  else  if 	(	(	(TIM5->CNT)<=	(10)	)		&&
//			( 	(TIM5->CNT)>=	(5) 	)			&&
//			(	cnt_old<=	(5)	)			&&
//			(	cnt_old>=0	)
//			  	  )
//  {
//
//	  laps--;
//  }
//  cnt_old=TIM5->CNT;

void CDC_ReceiveCallback(uint8_t *buf, uint32_t len)
{
	  for (int i=0;i<=16;i++)
	  {
		 rxbuf[i]=buf[i];
	  }
	  k=0;
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)==GPIO_PIN_RESET){
		  	  Xpos=0;
	 	  }
	 	  else {
	 		  Xpos=1;
	 	  }
	 	  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)==GPIO_PIN_RESET){
	 		  Xneg=0;
	 	  }
	 	  else  {
	 		  Xneg=1;
	 	  }
		  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)==GPIO_PIN_RESET){
			  Ypos=0;
		 	  }
		 	  else {
		 	 Ypos=1;
		 	  }
	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12)==GPIO_PIN_RESET){
		 	 Yneg=0;
		 	  }
		 	  else  {
		 	 Yneg=1;
		 	  }

	  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==GPIO_PIN_RESET){
		  	 Zpos=0;
	 	  }
	 	  else {
	 		 Zpos=1;
	 	  }

	  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)==GPIO_PIN_RESET){
		  	  Zneg=0;
	 	  }
	 	  else  {
	 		 Zneg=1;
	 	  }


	 	  if(highMode>1) {
		 	    xSpeed = 250U;
		       ySpeed = 250U;
		       xSen = 2U;
		       ySen = 2U;
		       highMode = 1;
	 	  }else if (highMode < 1) {


	 	    xSpeed = 50U;
	 	    ySpeed = 50U;
	 	    xSen = 10U;
	 	    ySen = 10U;
	 	    highMode = 1;
	 	  }

	 	  if(HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin) == GPIO_PIN_SET)
	 	      {
	 	  	    xPul = -(xSen);
	 	      }
//	 	  else if (HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin) == GPIO_PIN_SET)
//	 	      {
//	 	        xPul = (xSen);
//	 	      }
	 	  if (HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin) == GPIO_PIN_SET)
	 	  {
	 		  yPul = ySen;
	 	  }
//	 	  else if (HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin) == GPIO_PIN_SET)
//	 	  {
//	 		  yPul = -ySen;
//	 	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	int rec_state=0;
	  int8_t opbuff[8];


	  int read,control;
#define START 0
#define CMD 1
#define ADDRESS 2
#define DATA 3
#define CRC8 4



//	  if(buf[0]== 0X22)
//	  {
//	  *(int32_t*)&(opbuff[0]) = TIM5->CNT;
//
//	  opbuff[3] =(Xpos<<0)	+	(Xneg<<1)	+	(Ypos<<2)	+	(Yneg<<3)	+ 	(Zpos<<4)	+	(Zneg<<5);
//
//
//	  CDC_Transmit_FS(&opbuff[0], sizeof(int32_t));
//	  }
//
//	  else{
//		  opbuff[0]=0xFF;
//		  opbuff[1]=0xFF;
//		  opbuff[2]=0xFF;
//		  opbuff[3]=0xFF;
//		  CDC_Transmit_FS(&opbuff[0], sizeof(int32_t));
//	  }

  /* Infinite loop */
  for(;;)
  {
	  tim2Cnt=__HAL_TIM_GET_COUNTER(&htim2);
	  tim5Cnt=__HAL_TIM_GET_COUNTER(&htim5);

	  read=0;
	  control=0;

	  while (k < 16)
	  {
		  uint8_t ch = rxbuf[k];
		  switch (rec_state)
		  {
		  case START:
			  if (ch==0x22)
			  {
				  rec_state=CMD;
			  }
			  break;
		  case CMD:
			  if (ch==0x30)
			  {
				  rec_state=ADDRESS;
				  read=1;
			  }
			  else if (ch==0x66)
			  {
				  rec_state=ADDRESS;
				  control=1;
			  }
			  else
			  {
				  rec_state=START;
			  }
			  break;
		  case ADDRESS:
			  if(ch==0x00)
			  {
				  rec_state=DATA;
				  grating_scale_1=1;
			  }
			  else if (ch==0x01)
			  {
				  rec_state=DATA;
				  grating_scale_2=1;
			  }
			  else if (ch==0x02)
			  {
				  rec_state=DATA;
				  PGsend=1;
			  }
			  else if (ch==0x03)
			  {
				  rec_state=DATA;
				  spin=1;
			  }
			  else if (ch==0x04)
			  {
				  rec_state=DATA;
				  solder_push=1;
			  }
			  else
			  {
				  rec_state=START;
			  }
			  break;

		  case DATA:
			  if(grating_scale_1==1)
			  {
				  opbuff[0]=0x22;
				  opbuff[1]=0x30;
				  opbuff[2]=0x00;
				  *(int32_t*)&(opbuff[3]) = TIM5->CNT;
//				  opbuff[6]=0xFF;
				  opbuff[7]=0xFF;
				  CDC_Transmit_FS(&opbuff[0], 8);
			  }
			  else if(grating_scale_2==1)
			  {
				  opbuff[0]=0x22;
				  opbuff[1]=0x30;
				  opbuff[2]=0x01;
				  *(int32_t*)&(opbuff[3]) = TIM2->CNT;
//				  opbuff[6]=0xFF;
				  opbuff[7]=0xFF;
				  CDC_Transmit_FS(&opbuff[0], 8);
			  }
			  else if(PGsend==1)
			  {
				  opbuff[0]=0x22;
				  opbuff[1]=0x30;
				  opbuff[2]=0x02;
				  opbuff[3] =(Xpos<<0)	+	(Xneg<<1)	+	(Ypos<<2)	+	(Yneg<<3)	+ 	(Zpos<<4)	+	(Zneg<<5);
				  opbuff[4]=0xFF;opbuff[5]=0xFF;opbuff[6]=0xFF;opbuff[7]=0xFF;
				  CDC_Transmit_FS(&opbuff[0], 8);
			  }
			  else if(spin==1)
			  {
				  xPul=10U;
				  xSpeed = 50U;
				  spin_length=(rxbuf[k]) + (rxbuf[k+1]<<8);
			  }
			  else if (solder_push==1)
			  {
				  yPul=10U;
				  ySpeed = 50U;
				  push_length=(rxbuf[k]) + (rxbuf[k+1]<<8);
			  }
			  rec_state=CRC8;
			  break;
		  case CRC8:
			  grating_scale_1=0;
			  grating_scale_2=0;
			  PGsend=0;
			  read=0;
			  control=0;
			  spin=0;
			  solder_push=0;

			  rec_state=START;
			  break;
		  }
	  k++;
	  }


    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
