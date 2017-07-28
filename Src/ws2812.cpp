#include "ws2812.hpp"

Ws2812::Ws2812(uint8_t num, uint16_t outputPin, GPIO_TypeDef* outputPort) {

	pin = outputPin;
	port = outputPort;

	begun = false;

	if(pixels) free(pixels);

	int numBytes = numLEDs * 3;

	if((pixels = (uint8_t *)malloc(numBytes))) {
	    memset(pixels, 0, numBytes);
		numLEDs = num;
	} else {
	numLEDs = numBytes = 0;
	}

}

void Ws2812::begin(void) {

	if (pin == 0x00) {
		return;
	}

	/* Enable GPIO clock  */
	RCC->AHBENR |= (1 << ((uint32_t)(port - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE)) + 17));

	/* Initialise pin */
	uint8_t pinpos;

	/* Go through all pins */
	for (pinpos = 0; pinpos < 0x10; pinpos++) {

		if ((pin & (1 << pinpos)) == 0) {
			continue;
		}

		/* Set GPIO PUPD register, no pull up/down */
		port->PUPDR = (port->PUPDR & ~(0x03 << (2 * pinpos))) | ((uint32_t)(0x00 << (2 * pinpos)));

		/* Set GPIO MODE register, output */
		port->MODER = (port->MODER & ~((uint32_t)(0x03 << (2 * pinpos)))) | ((uint32_t)(0x01 << (2 * pinpos)));

		/* Set GPIO OTYPE register, push-pull */
		port->OTYPER = (port->OTYPER & ~(uint16_t)(0x01 << pinpos)) | ((uint16_t)(0x00 << pinpos));

		/* Set GPIO OSPEED register, fast-speed */
		port->OSPEEDR = (port->OSPEEDR & ~((uint32_t)(0x03 << (2 * pinpos)))) | ((uint32_t)(0x03 << (2 * pinpos)));
	}

	begun = true;

}

void Ws2812::show() {
	if(!pixels) return;

	volatile uint8_t *ptr = pixels;
	volatile uint8_t b = *ptr++;

	for(int i = 0; i < (numLEDs * 3); i++) {
		for(int j = 7; j >= 0; j--) {
				ws2812_writebit(((b >> j)  & 0x01));
		}

		b = *ptr++;
	}

}

void Ws2812::setPixelColor(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue) {
	if(pixel < numLEDs) {

		uint8_t *p;

		p = &pixels[pixel * 3];

		p[0] = green;
		p[1] = red;
		p[2] = blue;

	}
}

void Ws2812::setPixelColor(uint8_t pixel, uint32_t color) {
	if(pixel < numLEDs) {

		uint8_t *p,
		  r = (uint8_t)(color >> 16),
		  g = (uint8_t)(color >>  8),
		  b = (uint8_t)color;

		p = &pixels[pixel * 3];

		p[0] = g;
		p[1] = r;
		p[2] = b;

	}
}

void Ws2812::setAllColor(uint8_t red, uint8_t green, uint8_t blue) {

	for(int i = 0; i < numLEDs; i++) {
		setPixelColor(i,red,green,blue);
	}

}

void Ws2812::setAllColor(uint32_t color) {

	for(int i = 0; i < numLEDs; i++) {
		setPixelColor(i,color);
	}

}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t Ws2812::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

uint8_t Ws2812::numPixels(void) const {
	return numLEDs;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

inline void Ws2812::ws2812_writebit(uint8_t bit) {
	if(bit == 1) {
		ws2812_writebit_high();
	}
	else {
		ws2812_writebit_low();
	}
}

inline void Ws2812::ws2812_writebit_high() {

	port->BSRR = (uint32_t)(pin);


	//all delays found by trial-and-error, should be improved with proper delay function / timer
	/*
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
*/
	port->BRR = (uint32_t)(pin);
/*
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	*/
}

inline void Ws2812::ws2812_writebit_low() {

	port->BSRR = (uint32_t)(pin);
/*
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
*/
	port->BRR = (uint32_t)(pin);
/*
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	*/
}

#pragma GCC pop_options


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return Ws2812::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Ws2812::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Ws2812::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

