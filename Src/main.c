/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
//#include "ws2812b.h"
//#include "visEffect.h"

char RxData[11]={'0'};
uint8_t MyColor[3]={0};

#define UserLED_Toggle()	 HAL_GPIO_TogglePin(UserLED_GPIO_Port,UserLED_Pin)
#define UserLED_ON()  		 HAL_GPIO_WritePin(UserLED_GPIO_Port,UserLED_Pin,GPIO_PIN_SET)
#define UserLED_OFF() 		 HAL_GPIO_WritePin(UserLED_GPIO_Port,UserLED_Pin,GPIO_PIN_RESET)

#define numLEDs 9

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t sendButtonBuff[8]={0, 0, 0x04, 0, 0, 0, 0, 0};
uint8_t buttonColor[3][104];
uint8_t pixelDataFlow[1024+3*104*4];
uint8_t colorDataFlow[3*104] = {0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA };
uint16_t colorDataIndex = 0;
TIM_OC_InitTypeDef mysConfigOC;
//uint8_t pixelReset[1024];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern uint8_t USBD_HID_SendReport (USBD_HandleTypeDef *pdev, 
                                 uint8_t *report,
                                 uint16_t len);

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,100);

  return ch;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void Din_1(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 48);
}
void Din_0(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 14);
}

void Send_8bits(uint8_t dat) 
{   
		uint8_t i; 
		Din_0();
		for(i=0;i<8;i++)   
		{ 
			if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
			{      
				Din_1();				
			} 		
			else 	//0 ,for "0",H:0.4us,L:	0.85us			
			{ 
			 Din_0();					
			}
		   dat=dat<<1; 
	 }
}
//G--R--B
//MSB first	
void Send_2811_24bits(uint8_t GData,uint8_t RData,uint8_t BData)
 {   
	Send_8bits(GData);  
	Send_8bits(RData);  
	Send_8bits(BData);
 } 
 

void rst() 
{ 
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);	
	 //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
	 HAL_Delay (1);
}	

uint8_t AscillToNum(char mychar)
{
	uint8_t num=0;
	switch(mychar)
	{
		case '0':num=0;break;
		case '1':num=1;break;
		case '2':num=2;break;
		case '3':num=3;break;
		case '4':num=4;break;
		case '5':num=5;break;
		case '6':num=6;break;
		case '7':num=7;break;
		case '8':num=8;break;
		case '9':num=9;break;
	}
	return num;
}

typedef uint8_t  echoDeng_u8Type;

echoDeng_u8Type rBuffer[numLEDs]={0};
echoDeng_u8Type gBuffer[numLEDs]={0};
echoDeng_u8Type bBuffer[numLEDs]={0};

void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b)
{ 
				echoDeng_u8Type i=0;
				for(i=0;i<numLEDs;i++)
				{
					rBuffer[i]=r;
					gBuffer[i]=g;
					bBuffer[i]=b;
				}
				for(i=0;i<numLEDs;i++)
				{							  
						Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
				}
 }
 void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
 {	 
				echoDeng_u8Type i=0;
				rBuffer[n]=r;
				gBuffer[n]=g;
				bBuffer[n]=b;
				for(i=0;i<numLEDs;i++)
				{							  
						Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
				}
 }
  void SetPixelColor(uint16_t n, uint32_t c)
 {	 
				echoDeng_u8Type i=0;
				rBuffer[n]=(uint8_t)(c>>16);
				gBuffer[n]=(uint8_t)(c>>8);
				bBuffer[n]=(uint8_t)c;
				for(i=0;i<numLEDs;i++)
				{							  
						Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
				}
 }
void PixelUpdate()
{
	rst();
}
uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}
uint32_t Wheel(uint8_t WheelPos)
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) 
	{
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
//??
void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for(j=0; j<256; j++) 
	{
    for(i=0; i<numLEDs; i++)
		{
      SetPixelColor(i, Wheel((i+j) & 255));
    }
		PixelUpdate();
    HAL_Delay (wait);
  }
}
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) 
{
  uint16_t i, j;

  for(j=0; j<256*5; j++) 
	{ // 5 cycles of all colors on wheel
    for(i=0; i< numLEDs; i++) 
	 {
     SetPixelColor(i, Wheel(((i * 256 / numLEDs) + j) & 255));
    }
	  PixelUpdate();
    HAL_Delay (wait);
  }
}
//Theatre-style crawling lights.???
void theaterChase(uint32_t c, uint8_t wait) 
{
  for (int j=0; j<10; j++) 
	{  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) 
		{
      for (uint16_t i=0; i < numLEDs; i=i+3)
			{
        SetPixelColor(i+q, c);    //turn every third pixel on
      }
			PixelUpdate();
      HAL_Delay(wait);

      for (uint16_t i=0; i < numLEDs; i=i+3) 
			{
       SetPixelColor(i+q, 0);        //turn every third pixel off
      }
			PixelUpdate();
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) 
{
  for (int j=0; j < 256; j++) 
	{     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++)
		{
      for (uint16_t i=0; i < numLEDs; i=i+3) 
			{
        SetPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      PixelUpdate();

      HAL_Delay(wait);

      for (uint16_t i=0; i < numLEDs; i=i+3)
			{
        SetPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) 
{
	uint16_t i=0;
  for( i=0; i<numLEDs; i++) 
	{
    SetPixelColor(i, c);
    PixelUpdate();
    HAL_Delay(wait);
  }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	char hello[] = "Hello,World!\n";
	uint8_t i=0;
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
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim7);

	for(i = 0; i < 104; i++)
	{
		/*
		buttonColor[0][i] = i ;
		buttonColor[1][i] = i ;
		buttonColor[2][i] = i ;
		*/
		/*
		buttonColor[0][i] = ((i + 3) * 247 + 48) / 5 % 0xFF ;
		buttonColor[1][i] = ((i + 158) * 87 + 7) / 77 % 0xFF ;
		buttonColor[2][i] = ((i + 8) * 5 + 21) / 12 % 0xFF ;
		*/
		
		buttonColor[0][i] = 0x00 ;
		buttonColor[1][i] = 0x01 ;
		buttonColor[2][i] = 0x01 ;
		
	}
	
	uint16_t pixelIndex = 0;
	for(i = 0; i < 104; i++)
	{
		for(uint8_t countrgb = 0; countrgb < 3; countrgb ++)
		{
			pixelIndex = 1024 + i*3*4 + (countrgb*4);
			for(uint8_t count = 0; count < 4; count ++)
			{
				pixelDataFlow[pixelIndex] = 0x88;
				if(buttonColor[countrgb][i] & 1 << count*2)
				{
					pixelDataFlow[pixelIndex] += 0x60;
				}
				if(buttonColor[countrgb][i] & 1 << (count*2 + 1))
				{
					pixelDataFlow[pixelIndex] += 0x06;
				}
				pixelIndex++;
			}
		}
	}
	
	//colorDataFlow[3*3] = 0x55;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		sendButtonBuff[2] = 0x04;
		//HAL_Delay(100);
		sendButtonBuff[2] = 0x00;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		
		//HAL_SPI_Transmit(&hspi2, pixelReset, 1024, 1000);
		HAL_SPI_Transmit(&hspi2, pixelDataFlow, 1024+3*104*4, 1000);
		
		HAL_UART_Transmit(&huart2, (uint8_t *)hello, sizeof(hello), 1000);
/*
		// Some example procedures showing how to display to the pixels:
		colorWipe(Color(255, 0, 0), 50); // Red
		colorWipe(Color(0, 255, 0), 50); // Green
		colorWipe(Color(0, 0, 255), 50); // Blue
		
		// Send a theater pixel chase in...
		theaterChase(Color(127, 127, 127), 50); // White
		theaterChase(Color(127, 0, 0), 50); // Red
		theaterChase(Color(0, 0, 127), 50); // Blue

		rainbow(20);//??
		rainbowCycle(20);//??
		theaterChaseRainbow(50);//???
		
		rst();
		for (i = 0; i<255; i++)
			Din_1();
		for (i = 0; i<255; i++)
		{
			//Din_0();
			Din_1();
		}
		*/
		
		HAL_Delay(1000);
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
		//HAL_TIM_Base_Start_IT(&htim2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

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

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 47;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
