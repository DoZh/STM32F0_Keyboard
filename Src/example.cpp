/*
 *	Written by Chris Barr, 2016.
 */

#include "ws2812.hpp"

#define WS2812_PORT GPIOA
#define WS2812_PIN GPIO_PIN_9
#define WS2812_NUM 8

Ws2812 pixels = Ws2812(WS2812_NUM,WS2812_PIN,WS2812_PORT);

int main(void)
{
	//setup
	HAL_Init();
	SystemClock_Config();

	HAL_Delay(50);

	pixels.begin();

	//infinite loop
	for(;;) {

		pixels.setPixelColor(0,255,0,0);		//Red
		pixels.setPixelColor(1,0,255,0);		//Green
		pixels.setPixelColor(2,0,0,255);		//Blue
		pixels.setPixelColor(3,127,127,0);		//Yellow
		pixels.setPixelColor(4,0,127,127);		//Cyan
		pixels.setPixelColor(5,127,0,127);		//Pink
		pixels.setPixelColor(6,100,100,100);	//White	

		pixels.show();

		HAL_Delay(1000);

		//Rainbow effect
		for(uint16_t j=0; j<256*6; j++) { // 6 cycles of all colors on wheel
			for(uint8_t i=0; i< pixels.numPixels(); i++) {
				pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
			}
			pixels.show();

			HAL_Delay(5);
		}
	}

}


/** System Clock Configuration, generated using CubeMX software.
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
