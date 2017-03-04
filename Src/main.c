/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "can.h"
#include "serial.h"
#include "nodeMiscHelpers.h"
#include "nodeConf.h"
#include "../../CAN_ID.h"

// RTOS Task functions + helpers
#include "Can_Processor.h"

//LTC6804
#include "LTC68041.h"

//MCP3909
#include "mcp3909.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId ApplicationHandle;
osThreadId Can_ProcessorHandle;
osThreadId RTHandle;
osThreadId SMTHandle;
osThreadId TMTHandle;
osMessageQId mainCanTxQHandle;
osMessageQId mainCanRxQHandle;
osTimerId WWDGTmrHandle;
osTimerId HBTmrHandle;
osMutexId swMtxHandle;
osSemaphoreId mcp3909_DRHandle;
osSemaphoreId mcp3909_RXHandle;
osSemaphoreId bmsTRxCompleteHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
bmsChainHandleTypeDef hbms1;

MCP3909HandleTypeDef hmcp1;
uint8_t mcpRxBuf[REG_LEN * REGS_NUM];
uint8_t mcpTxBuf[REG_LEN * REGS_NUM + CTRL_LEN];

// SW_Sentinel will fail the CC firmware check and result in node addition failure!
const uint32_t firmwareString = 0x00000001;			// Firmware Version string
const uint8_t selfNodeID = bps_nodeID;					// The nodeID of this node
uint32_t selfStatusWord = INIT;							// Initialize
#define NODE_CONFIGURED

#ifndef NODE_CONFIGURED
#error "NODE NOT CONFIGURED. GO CONFIGURE IT IN NODECONF.H!"
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
void doApplication(void const * argument);
void doProcessCan(void const * argument);
void doRT(void const * argument);
void doSMT(void const * argument);
void doTMT(void const * argument);
void TmrKickDog(void const * argument);
void TmrSendHB(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// Data Ready pin triggered callback (PA1)
void HAL_GPIO_EXTI_Callback(uint16_t pinNum){
	if(pinNum == GPIO_DR_Pin){
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		mcp3909_readAllChannels(&hmcp1,hmcp1.pRxBuf);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	// Check which SPI issued interrupt
	if(hspi == (hbms1.hspi)){
		HAL_GPIO_WritePin(BMS_CS_GPIO_Port,BMS_CS_Pin, GPIO_PIN_SET);
		xSemaphoreGiveFromISR(bmsTRxCompleteHandle, NULL);
	}else if(hspi == (hmcp1.hspi)){
		HAL_GPIO_WritePin(MCP_CS_GPIO_Port,MCP_CS_Pin, GPIO_PIN_SET);
		xSemaphoreGiveFromISR(mcp3909_RXHandle, NULL);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	// Check which SPI issued interrupt
	if(hspi == (hbms1.hspi)){
		HAL_GPIO_WritePin(BMS_CS_GPIO_Port,BMS_CS_Pin, GPIO_PIN_SET);
	}else if(hspi == (hmcp1.hspi)){
		HAL_GPIO_WritePin(MCP_CS_GPIO_Port,MCP_CS_Pin, GPIO_PIN_SET);
	}
}

void EM_Init(){
	hmcp1.phase[0] = 0;
	hmcp1.phase[1] = 0;
	hmcp1.phase[2] = 0;

	for(uint8_t i= 0; i < MAX_CHANNEL_NUM; i++){
		hmcp1.channel[i].PGA = PGA_1;
		hmcp1.channel[i].boost = BOOST_OFF;
		hmcp1.channel[i].dither = DITHER_ON;
		hmcp1.channel[i].reset = RESET_OFF;
		hmcp1.channel[i].shutdown = SHUTDOWN_OFF;
		hmcp1.channel[i].resolution = RES_16;
	}

	hmcp1.extCLK = 0;
	hmcp1.extVREF = 0;
	hmcp1.hspi = &hspi1;
	hmcp1.osr = OSR_32;
	hmcp1.prescale = PRESCALE_1;
	hmcp1.readType = READ_TYPE;

	hmcp1.pRxBuf = mcpRxBuf;
	hmcp1.pTxBuf = mcpTxBuf;

	HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0); // set DR pin interrupt priority
	mcp3909_init(&hmcp1);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
#define DISABLE_RT
//#define DISABLE_SMT
#define DISABLE_TMT
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  Serial2_begin();
  static uint8_t hbmsg[] = "Booting... \n";
  Serial2_writeBuf(hbmsg);

  ////*IF YOU GET HCAN1 NOT DEFINED ERROR, CHECK NODECONF.H FIRST!*////
  bxCan_begin(&hcan1, &mainCanRxQHandle, &mainCanTxQHandle);
  // TODO: Set node-specific CAN filters
  bxCan_addMaskedFilterStd(0,0,0); // Filter: Status word group (ignore nodeID)
  bxCan_addMaskedFilterExt(0,0,0);

#ifndef DISABLE_SMT
  /*
   * LTC68041 SETUP
   */
//  Set up the global ADC configs for the LTC6804
//  ltc68041ChainInitStruct bmsInitParams[TOTAL_IC];
//  LTC68041_Initialize(&hbms1, bmsInitParams);
  hbms1.hspi = &hspi3;
  if(ltc68041_Initialize(&hbms1) != 0){
	  for(;;);
  }
  HAL_WWDG_Refresh(&hwwdg);
#endif

#ifndef DISABLE_RT
  EM_Init();
  HAL_WWDG_Refresh(&hwwdg);
#endif
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of swMtx */
  osMutexDef(swMtx);
  swMtxHandle = osMutexCreate(osMutex(swMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of mcp3909_DR */
  osSemaphoreDef(mcp3909_DR);
  mcp3909_DRHandle = osSemaphoreCreate(osSemaphore(mcp3909_DR), 1);

  /* definition and creation of mcp3909_RX */
  osSemaphoreDef(mcp3909_RX);
  mcp3909_RXHandle = osSemaphoreCreate(osSemaphore(mcp3909_RX), 1);

  /* definition and creation of bmsTRxComplete */
  osSemaphoreDef(bmsTRxComplete);
  bmsTRxCompleteHandle = osSemaphoreCreate(osSemaphore(bmsTRxComplete), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of WWDGTmr */
  osTimerDef(WWDGTmr, TmrKickDog);
  WWDGTmrHandle = osTimerCreate(osTimer(WWDGTmr), osTimerPeriodic, NULL);

  /* definition and creation of HBTmr */
  osTimerDef(HBTmr, TmrSendHB);
  HBTmrHandle = osTimerCreate(osTimer(HBTmr), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(WWDGTmrHandle, WD_Interval);
  osTimerStart(HBTmrHandle, HB_Interval);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Application */
  osThreadDef(Application, doApplication, osPriorityNormal, 0, 512);
  ApplicationHandle = osThreadCreate(osThread(Application), NULL);

  /* definition and creation of Can_Processor */
  osThreadDef(Can_Processor, doProcessCan, osPriorityBelowNormal, 0, 512);
  Can_ProcessorHandle = osThreadCreate(osThread(Can_Processor), NULL);

  /* definition and creation of RT */
  osThreadDef(RT, doRT, osPriorityHigh, 0, 512);
  RTHandle = osThreadCreate(osThread(RT), NULL);

  /* definition and creation of SMT */
  osThreadDef(SMT, doSMT, osPriorityAboveNormal, 0, 512);
  SMTHandle = osThreadCreate(osThread(SMT), NULL);

  /* definition and creation of TMT */
  osThreadDef(TMT, doTMT, osPriorityAboveNormal, 0, 512);
  TMTHandle = osThreadCreate(osThread(TMT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of mainCanTxQ */
  osMessageQDef(mainCanTxQ, 32, Can_frame_t);
  mainCanTxQHandle = osMessageCreate(osMessageQ(mainCanTxQ), NULL);

  /* definition and creation of mainCanRxQ */
  osMessageQDef(mainCanRxQ, 16, Can_frame_t);
  mainCanRxQHandle = osMessageCreate(osMessageQ(mainCanRxQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_3TQ;
  hcan1.Init.BS1 = CAN_BS1_12TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = ENABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 50585;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 16;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
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

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BPS_KILL_Pin|BPS_FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCP_CS_Pin|BMS_CS_Pin|MUX_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MUX_SEL0_Pin|MUX_SEL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUX_SEL2_Pin|MUX_SEL3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BPS_KILL_Pin BPS_FAN_Pin */
  GPIO_InitStruct.Pin = BPS_KILL_Pin|BPS_FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_DR_Pin */
  GPIO_InitStruct.Pin = GPIO_DR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_DR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP_CS_Pin */
  GPIO_InitStruct.Pin = MCP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MCP_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_CS_Pin */
  GPIO_InitStruct.Pin = BMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BMS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_EN_Pin */
  GPIO_InitStruct.Pin = MUX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUX_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_SEL0_Pin MUX_SEL1_Pin */
  GPIO_InitStruct.Pin = MUX_SEL0_Pin|MUX_SEL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_SEL2_Pin MUX_SEL3_Pin */
  GPIO_InitStruct.Pin = MUX_SEL2_Pin|MUX_SEL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* doApplication function */
void doApplication(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* doProcessCan function */
void doProcessCan(void const * argument)
{
  /* USER CODE BEGIN doProcessCan */
  /* Infinite loop */
	for(;;){
		// Wrapper function for the CAN Processing Logic
		// Handles all CAN Protocol Suite based responses and tasks
		Can_Processor();
	}
  /* USER CODE END doProcessCan */
}

/* doRT function */
void doRT(void const * argument)
{
  /* USER CODE BEGIN doRT */
#ifndef DISABLE_RT
  /* Infinite loop */
  for(;;)
  {
	  if((selfStatusWord & 0x07) == ACTIVE){
		mcp3909_wakeup(&hmcp1);
		xSemaphoreTake(mcp3909_RXHandle, portMAX_DELAY);
		mcp3909_parseChannelData(&hmcp1);
		// XXX: Energy metering algorithm
		mcp3909_sleep(&hmcp1);

		osDelay(RT_Interval);
	  }else{
		  osDelay(1);
	  }
  }
#else
  for(;;){
	  osDelay(1000);
  }
#endif
  /* USER CODE END doRT */
}

/* doSMT function */
void doSMT(void const * argument)
{
  /* USER CODE BEGIN doSMT */
#ifndef DISABLE_SMT

	static Can_frame_t newFrame;
	newFrame.dlc = 8;
	newFrame.isExt = 0;
	newFrame.isRemote = 0;

  /* Infinite loop */
  for(;;)
  {
	  if((selfStatusWord & 0x07) == ACTIVE){
		  int8_t success = ltc68041_clearCell(&hbms1);
		  osDelay(3);
		  success = ltc68041_startCVConv(&hbms1);

		  // Delay enough time but also make sure that the chip doesn't go into sleep mode
		for(uint8_t i = 0; i < 3 * TOTAL_IC; i++){
			osDelay(3);
			wakeup_sleep();
		}

		// Read the register groups
		success = ltc68041_readRegGroup(&hbms1, RDCVA);
		osDelay(2);
		ltc68041_parseCV(&hbms1, A);

		success = ltc68041_readRegGroup(&hbms1, RDCVB);
		osDelay(2);
		ltc68041_parseCV(&hbms1, B);

		success = ltc68041_readRegGroup(&hbms1, RDCVC);
		osDelay(2);
		ltc68041_parseCV(&hbms1, C);

		success = ltc68041_readRegGroup(&hbms1, RDCVD);
		osDelay(2);
		ltc68041_parseCV(&hbms1, D);

//		success = ltc68041_readRegGroup(&hbms1, RDSTATB);
//		osDelay(2);
//		ltc68041_parseSTAT(&hbms1, B);

#define vovTo100uV(x) ((x+1)*16)

		static uint8_t ohno[] = "oh no!\n";

		for(uint8_t i=0; i<3; i++){
			for(uint8_t j=0; j<12; j+=4){
				newFrame.id = voltOffset+i*3+j/4;
				newFrame.Data[0] = hbms1.board[i].CVR[j+0] >> 8;
				newFrame.Data[1] = hbms1.board[i].CVR[j+0] & 0xff;
				newFrame.Data[2] = hbms1.board[i].CVR[j+1] >> 8;
				newFrame.Data[3] = hbms1.board[i].CVR[j+1] & 0xff;
				newFrame.Data[4] = hbms1.board[i].CVR[j+2] >> 8;
				newFrame.Data[5] = hbms1.board[i].CVR[j+2] & 0xff;
				newFrame.Data[6] = hbms1.board[i].CVR[j+3] >> 8;
				newFrame.Data[7] = hbms1.board[i].CVR[j+3] & 0xff;
				if(bxCan_sendFrame(&newFrame) != 0){
					Serial2_writeBuf(ohno);
				}
//				static uint8_t msg[3];
//				msg[0] = hbms1.board[i].CVR[j+0] >> 8;
//				msg[1] = hbms1.board[i].CVR[j+0] & 0xff;
//				Serial2_writeBuf(msg);
				for(uint8_t k=0; k<4; k++){
					if(hbms1.board[i].CVR[j+k] > vovTo100uV(VOV) || hbms1.board[i].CVR[j+k] < vovTo100uV(VUV)){
						Serial2_writeBuf(ohno);
						assert_bps_fault(i*3+j/4, hbms1.board[i].CVR[j+k]);
					}
				}
			}
			osDelay(1);
		}

		//Check OV UV flags
//		for(int i=0; i<3; i++){
//			if(hbms1.board[i].STATR[5] || (hbms1.board[i].STATR[6] & 0xff)){
//				assert_bps_fault(i,0);
//			}
//		}

		osDelay(SMT_Interval - (8+TOTAL_IC*4));
	  }else{
		  osDelay(1);
	  }
  }
#else
  for(;;){
	  osDelay(1000);
  }
#endif
  /* USER CODE END doSMT */
}

/* doTMT function */
void doTMT(void const * argument)
{
  /* USER CODE BEGIN doTMT */
#ifndef DISABLE_TMT
  /* Infinite loop */
  for(;;)
  {
	  if((selfStatusWord & 0x07) == ACTIVE){
	  //Michael Pls
		  osDelay(TMT_Interval);
	  }else{
		  osDelay(1);
	  }
  }

#else
  for(;;){
	  osDelay(1000);
  }
#endif
  /* USER CODE END doTMT */
}

/* TmrKickDog function */
void TmrKickDog(void const * argument)
{
  /* USER CODE BEGIN TmrKickDog */
	// CHECKED
	taskENTER_CRITICAL();
	HAL_WWDG_Refresh(&hwwdg);
	taskEXIT_CRITICAL();
  /* USER CODE END TmrKickDog */
}

/* TmrSendHB function */
void TmrSendHB(void const * argument)
{
  /* USER CODE BEGIN TmrSendHB */
	// CHECKED
	static Can_frame_t newFrame;
	newFrame.isExt = 0;
	newFrame.isRemote = 0;

	if(getSelfState() == ACTIVE){
		// Assemble new heartbeat frame
		newFrame.id = selfNodeID + swOffset;
		newFrame.dlc = CAN_HB_DLC;

		#ifdef DEBUG
			static uint8_t hbmsg[] = "Heartbeat issued\n";
			Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
		#endif
	}
	else if (getSelfState() == INIT){
		// Assemble new addition request (firmware version) frame
		newFrame.id = selfNodeID + fwOffset;
		newFrame.dlc = CAN_FW_DLC;

		#ifdef DEBUG
			static uint8_t hbmsg[] = "Init handshake issued\n";
			Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
		#endif
	}
	bxCan_sendFrame(&newFrame);
	// No heartbeats sent in other states
  /* USER CODE END TmrSendHB */
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
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
