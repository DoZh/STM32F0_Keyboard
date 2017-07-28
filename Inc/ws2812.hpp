/*
 *	WS2812 / NeoPixel implementation for STM32F0x microcontrollers.
 *	Assumes a 48MHz clock rate from the internal oscillator / PLL.
 *
 *  Likely to work with other STM32Fxx, provided clock rate is similar
 *  or timing is adjusted to suit.
 *
 *  Based off of the Adafruit NeoPixel library for Arduino.
 *
 *	This is a VERY basic implementation that relies on bit-banging the signal
 *	to the WS2812 LEDs, and the output waveforms are very rough - in some situations
 *	the signal may result in incorrect behaviour.
 *
 *	A better approach would use DMA to write out values to the LEDs - but this is
 *	what I have at the moment, hopefully it helps someone else get started.
 *
 * Example usage (see included example file for full example):
 *
 * 			#define WS2812_NUM 16
 * 			#define WS2812_PIN GPIO_PIN_5
 * 			#define WS2812_PORT GPIOA
 *
 * 			Ws2812 pixels = Ws2812(WS2812_NUM,WS2812_PIN,WS2812_PORT);
 *
 * 			pixels.begin();
 *
 * 			for(uint8_t i=0; i< pixels.numPixels(); i++) {
 *				pixels.setPixelColor(i, 100,100,100);
 *			}
 *
 *	Written by Chris Barr, 2016.
 */

#ifndef __ws2812
#define __ws2812

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

uint32_t Wheel(uint8_t WheelPos);

class Ws2812 {
public:
	Ws2812(uint8_t num, uint16_t pin, GPIO_TypeDef* port);

	void begin(void);
	void show(void);
	void setPixelColor(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue);
	void setPixelColor(uint8_t pixel, uint32_t color);

	//helper functions to deal with entire strip at once
	void setAllColor(uint8_t red, uint8_t green, uint8_t blue);
	void setAllColor(uint32_t color);

	uint8_t numPixels(void) const;

	static uint32_t Color(uint8_t r, uint8_t g, uint8_t b);


private:

	bool begun;
	uint8_t *pixels;
	uint8_t numLEDs;

	uint16_t pin;
	GPIO_TypeDef* port;

	inline void ws2812_writebit(uint8_t bit);
	inline void ws2812_writebit_high(void);
	inline void ws2812_writebit_low(void);
};

#endif
