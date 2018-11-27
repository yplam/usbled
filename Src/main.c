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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <usbd_cdc_if.h>
#include <circ_buf.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);

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

CIRC_BUF_DEF(cdc_circ_buf, 2048);
volatile static uint8_t needUpdate = 0;
#define LED_COUNT 13
uint8_t ledSpiBuf[LED_COUNT * 3] = {0};
uint8_t ledPWMBuf[(LED_COUNT+50) * 3 * 8] = {0};
uint8_t * ledPWMDataPtr = 0;
uint8_t hi, lo, chk, i;
AppState_t appState;
uint16_t ledLen;
uint16_t ledBufIndex = 0;
uint32_t last_data_tick;

void fillLedPwmBuff(int ledx, uint8_t value);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ledPWMDataPtr = &ledPWMBuf[50];
  for(i=0; i<50; i++){
    ledPWMBuf[i] = 0x00;
  }

  HAL_Delay(1000);
  for (i = 0; i < LED_COUNT; i++) {
    ledSpiBuf[i * 3] = 0x01;
    ledSpiBuf[i * 3 + 1] = 0x00;
    ledSpiBuf[i * 3 + 2] = 0x00;

    fillLedPwmBuff(i * 3, 0x01);
    fillLedPwmBuff(i * 3 + 1, 0x00);
    fillLedPwmBuff(i * 3 + 2, 0x00);
  }
  fillLedPwmBuff(i * 3, 0x00);
  fillLedPwmBuff(i * 3+1, 0x00);

//  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &ledPWMBuf[0], (uint16_t)(LED_COUNT));
//  HAL_TIM_OC_Start_DMA(&htim2, TIM_CHANNEL_1, &ledPWMBuf[0], (uint16_t)(LED_COUNT * 3 * 8));
  HAL_SPI_Transmit_DMA(&hspi2, &ledPWMBuf[0], sizeof(ledPWMBuf));
  HAL_SPI_Transmit(&hspi1, &ledSpiBuf[0], LED_COUNT * 3, HAL_MAX_DELAY);
  HAL_Delay(1000);
  for (i = 0; i < LED_COUNT; i++) {
    ledSpiBuf[i * 3] = 0x00;
    ledSpiBuf[i * 3 + 1] = 0x00;
    ledSpiBuf[i * 3 + 2] = 0x00;

    fillLedPwmBuff(i * 3, 0x00);
    fillLedPwmBuff(i * 3 + 1, 0x00);
    fillLedPwmBuff(i * 3 + 2, 0x00);
  }
//  HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &ledPWMBuf[0], (uint16_t)(LED_COUNT));
  HAL_SPI_Transmit_DMA(&hspi2, &ledPWMBuf[0], sizeof(ledPWMBuf));
//  HAL_SPI_Transmit_DMA(&hspi1, &ledSpiBuf[0], LED_COUNT * 3);
  HAL_SPI_Transmit(&hspi1, &ledSpiBuf[0], LED_COUNT * 3, HAL_MAX_DELAY);
  HAL_Delay(1000);
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
          ledLen = (uint16_t) (3L * (256L * (long) hi + (long) lo));
          ledBufIndex = 0;
          appState = APP_SATTE_DATA;
          HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
        } else {
          appState = APP_SATTE_INIT;

        }
        break;
      case APP_SATTE_DATA:
        ledSpiBuf[ledBufIndex] = tmp;
        fillLedPwmBuff(ledBufIndex, tmp);
        ledBufIndex++;
        if (ledBufIndex == ledLen) {
//          HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &ledPWMBuf[0], (uint16_t)(ledLen*8));
          HAL_SPI_Transmit_DMA(&hspi2, &ledPWMBuf[0], (uint16_t)((ledLen+50*3)*8));
          HAL_SPI_Transmit(&hspi1, &ledSpiBuf[0], ledLen, HAL_MAX_DELAY);
//          HAL_SPI_Transmit_DMA(&hspi1, &ledSpiBuf[0], ledLen);
          appState = APP_SATTE_INIT;
        }
        break;
    }
//    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);

//    uint16_t i=0;
//    for(i=0; i< 10; i++){
//      if(-1 == circ_buf_pop(&cdc_circ_buf, &buf[i])){
//        break;
//      }
//      if(buf[i] == '\n'){
//        i++;
//        buf[i] = '\r';
//      }
//      else if(buf[i] == '\r'){
//        i++;
//        buf[i] = '\n';
//      }
//    }
//    if(i > 0){
//      CDC_Transmit_FS(buf, i);
//    }
//    for(i=0; i<39; i++){
//      ledBuf[i] = (uint8_t)i;
//    }
//    if(needUpdate){
//      needUpdate = 0;
//      HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
//      HAL_SPI_Transmit(&hspi1, &ledBuf[0], LED_COUNT*3, HAL_MAX_DELAY);
//      HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
//    }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void onSerial(uint8_t *Buf, uint32_t Len) {
  for (uint32_t i = 0; i < Len; i++) {
    circ_buf_push(&cdc_circ_buf, Buf[i]);
  }
}

void fillLedPwmBuff(int ledx, uint8_t value) {
  int i;
  if (ledx < LED_COUNT*3) {
    if(ledx % 3 == 2){
      ledx = ledx - 2;
    }
    else{
      ledx = ledx + 1;
    }
    for (i = 0; i < 8; i++) {
      ledPWMDataPtr[ledx*8 + i] = (value & (1 << (7 - i))) ? (uint8_t)0xF8 : (uint8_t)0xE0;
    }
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
