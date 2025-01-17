/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <usbd_cdc_if.h>
#include <circ_buf.h>
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
typedef enum {
    APP_SATTE_INIT,
    APP_SATTE_INIT_D,
    APP_SATTE_INIT_A,
    APP_SATTE_HI,
    APP_SATTE_LO,
    APP_SATTE_CHK,
    APP_SATTE_DATA
} AppState_t;

CIRC_BUF_DEF(cdc_circ_buf, 1024);
volatile static uint8_t needUpdate = 0;
#define LED_COUNT 1000
uint8_t ledBuf[LED_COUNT * 3] = {0};
#define LED_PWM_HALF_LEN 50
#define LED_PWM_TOTAL_LEN (LED_PWM_HALF_LEN*2)
#define PWM_BUFF_LEN  (LED_PWM_TOTAL_LEN * 3 * 8)
uint8_t ledPWMBuf[PWM_BUFF_LEN] = {0};

uint16_t ledPWMBufIndex;   // 下次DMA数据写入到 ledPWMBuf 的 Index
uint16_t ledPWMSendIndex;  //  ledBuf中下一个写入到 ledPWMBuf 进行数据传输的 Index
static uint8_t hi, lo, chk;
static AppState_t appState;
static uint16_t ledLen = LED_COUNT*3;
static uint16_t ledBufIndex = 0;
static uint32_t last_data_tick;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void APP_PWM_Start_DMA(void);
void APP_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void APP_PWM_Fill_Half(void);
void fillLedPwmBuff(int ledx, uint8_t value);
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
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  uint16_t i;
  for (i = 0; i < LED_COUNT; i++) {
    ledBuf[i * 3] = 0x00;
    ledBuf[i * 3 + 1] = 0x00;
    ledBuf[i * 3 + 2] = 0x01;
  }

//  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&ledPWMBuf[0], PWM_BUFF_LEN);
  APP_PWM_Start_DMA();
  HAL_SPI_Transmit_DMA(&hspi1, &ledBuf[0], LED_COUNT * 3);
  HAL_Delay(500);

  for (i = 0; i < LED_COUNT; i++) {
    ledBuf[i * 3] = 0x00;
    ledBuf[i * 3 + 1] = 0x00;
    ledBuf[i * 3 + 2] = 0x00;
  }
//  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&ledPWMBuf[0], PWM_BUFF_LEN);
  APP_PWM_Start_DMA();
  HAL_SPI_Transmit_DMA(&hspi1, &ledBuf[0], LED_COUNT * 3);
  HAL_Delay(10);

  uint8_t buf[12];

  buf[0] = 'A';
  buf[1] = 'd';
  buf[2] = 'a';
  buf[3] = '\n';
  CDC_Transmit_FS(buf, 4);
  last_data_tick = HAL_GetTick();

  uint8_t tmp;
  int tmp_result;
  uint32_t cur_tick;
  appState = APP_SATTE_INIT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    cur_tick = HAL_GetTick();
    if (cur_tick - last_data_tick > 15000) {
      buf[0] = 'A';
      buf[1] = 'd';
      buf[2] = 'a';
      buf[3] = '\n';
      appState = APP_SATTE_INIT;
      CDC_Transmit_FS(buf, 4);
      last_data_tick = HAL_GetTick();
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
    }
    tmp_result = circ_buf_pop(&cdc_circ_buf, &tmp);
    if (-1 == tmp_result) {
      continue;
    }
    last_data_tick = cur_tick;
    switch (appState) {
      case APP_SATTE_INIT:
        if (tmp == 'A') {
          appState = APP_SATTE_INIT_D;
        }
        break;
      case APP_SATTE_INIT_D:
        if (tmp == 'd') {
          appState = APP_SATTE_INIT_A;

        } else {
          appState = APP_SATTE_INIT;
        }
        break;
      case APP_SATTE_INIT_A:
        if (tmp == 'a') {
          appState = APP_SATTE_HI;

        } else {
          appState = APP_SATTE_INIT;
        }
        break;
      case APP_SATTE_HI:
        hi = tmp;
        appState = APP_SATTE_LO;
        break;
      case APP_SATTE_LO:
        lo = tmp;
        appState = APP_SATTE_CHK;
        break;
      case APP_SATTE_CHK:
        chk = tmp;

        if (chk == (hi ^ lo ^ 0x55)) {
          ledLen = (uint16_t) (3L * (256L * (long) hi + (long) lo + 1));
          ledBufIndex = 0;
          appState = APP_SATTE_DATA;
          HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
        } else {
          appState = APP_SATTE_INIT;

        }
        break;
      case APP_SATTE_DATA:
        ledBuf[ledBufIndex] = tmp;
        ledBufIndex++;
        if (ledBufIndex == ledLen) {
//          HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&ledPWMBuf[0], (uint16_t)((LED_PWM_RESET_LEN*3+ledLen)*8));
          APP_PWM_Start_DMA();
          HAL_SPI_Transmit_DMA(&hspi1, &ledBuf[0], ledLen);
          appState = APP_SATTE_INIT;
        }
        break;
    }
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 19;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  htim2.hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = APP_TIM_DMADelayPulseCplt;
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void onSerial(uint8_t *Buf, uint32_t Len) {
  for (uint32_t i = 0; i < Len; i++) {
    circ_buf_push(&cdc_circ_buf, Buf[i]);
  }
}

void fillLedPwmReset(int ledx) {
  for (uint16_t i = 0; i < 8; i++) {
    ledPWMBuf[ledx*8 + i] = 0x00;
  }
}

void fillLedPwmBuff(int ledx, uint8_t value) {
  uint16_t i;
  if(ledx % 3 == 2){
    ledx = ledx - 2;
  }
  else{
    ledx = ledx + 1;
  }
  for (i = 0; i < 8; i++) {
    ledPWMBuf[ledx*8 + i] = (value & (1 << (7 - i))) ? (uint8_t)(2*TIM2->ARR/3) : (uint8_t)(TIM2->ARR/3);
  }
}

void APP_PWM_Start_DMA(void) {
//  ledPWMBufIndex;   // 下次DMA数据写入到 ledPWMBuf 的 Index
//  ledPWMSendIndex;  //  ledBuf中下一个写入到 ledPWMBuf 进行数据传输的 Index
  ledPWMBufIndex = LED_PWM_HALF_LEN*3;
  ledPWMSendIndex = 0;
  // RESET 码
  uint16_t i = 0;
  for(i=0; i<LED_PWM_HALF_LEN*3; i++){
    fillLedPwmReset(i);
  }
  APP_PWM_Fill_Half();
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&ledPWMBuf[0], (uint16_t)(PWM_BUFF_LEN));
}

void APP_PWM_Fill_Half(void) {
  uint16_t i = 0;
  for(i=0; i<LED_PWM_HALF_LEN*3; i++){
    if(ledPWMSendIndex >= ledLen){
      break;
    }
    fillLedPwmBuff(ledPWMBufIndex+i, ledBuf[ledPWMSendIndex]);
    ledPWMSendIndex ++;
  }
  while(i<LED_PWM_HALF_LEN*3){
    fillLedPwmReset(i);
    i++;
  }
  if(ledPWMBufIndex == 0){
    ledPWMBufIndex = LED_PWM_HALF_LEN*3;
  }
  else{
    ledPWMBufIndex = 0;
  }
}

void APP_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma) {
  APP_PWM_Fill_Half();
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance==TIM2)
  {
    if(ledPWMSendIndex >= ledLen){
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
      HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    }
    else{
      APP_PWM_Fill_Half();
    }
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
