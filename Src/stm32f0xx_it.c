/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "usbd_hid.h"
#include "usb_device.h"
extern uint8_t colorDataFlow[3*104];
extern uint16_t colorDataIndex;
extern uint8_t sendButtonBuff[8];
extern SPI_HandleTypeDef hspi1;
extern uint8_t buttonDataFlow[16];
extern uint8_t buttonLookupTable[128];
uint8_t prepareStop = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
	
	if((colorDataIndex++) < 8*3*104)
	{
	if(colorDataFlow[colorDataIndex >> 8] & 1 << (colorDataIndex % 8))
	//if(colorDataIndex % 2)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 48);
			GPIOA->BSRR = (uint32_t)GPIO_PIN_1;
		}
	else
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 14);
			GPIOA->BRR = (uint32_t)GPIO_PIN_1;
		}
	}
	else
	{
		if (prepareStop++ < 2)
		{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		}
		else
		{
			//HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
			colorDataIndex = 0 ;
			prepareStop = 0;
		}
	}
  /* USER CODE END TIM2_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
	//HAL_TIM_PeriodElapsedCallback(&htim2);
	
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

	uint8_t countbyte,countbit,buttonDataBuf,keycount = 0;
	GPIOA->BRR = (uint32_t)GPIO_PIN_4;
	HAL_SPI_Receive(&hspi1, buttonDataFlow, 16, 1000);
	GPIOA->BSRR = (uint32_t)GPIO_PIN_4;
	//for(countbyte = 0; countbyte < 16; countbyte++)
	for(countbyte = 0; countbyte < 2; countbyte++)
	{
		buttonDataBuf = 0xFF - buttonDataFlow[countbyte];
		if (buttonDataBuf)
			for(countbit = 0; countbit < 8; countbit++)
			{
				if(buttonDataBuf & 1 << countbit && keycount <= 6)
				{
					sendButtonBuff[2 + keycount++] = buttonLookupTable[(uint16_t)(countbyte * 8) + countbit];
				}
			}
	}
	for(;keycount <= 6; keycount++)
		sendButtonBuff[2 + keycount] = 0;

	USBD_HID_SendReport(&hUsbDeviceFS, sendButtonBuff, 8);
	
  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles USB global interrupt / USB wake-up interrupt through EXTI line 18.
*/
void USB_IRQHandler(void)
{
  /* USER CODE BEGIN USB_IRQn 0 */

  /* USER CODE END USB_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_IRQn 1 */

  /* USER CODE END USB_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
