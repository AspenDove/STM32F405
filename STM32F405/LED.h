#pragma once

#include <stm32f4xx_hal.h>
#include "gpio.h"

class LED
{
public:
	void Init(const reuse& r)
	{
		this->r = r;
		GPIO_Init(r.GPIOx, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, r.Pin);
	}
	void on()
	{
		HAL_GPIO_WritePin(r.GPIOx, r.Pin, GPIO_PIN_RESET);
	}
	void off()
	{
		HAL_GPIO_WritePin(r.GPIOx, r.Pin, GPIO_PIN_SET);
	}
private:
	reuse r{};
};

extern LED led1, led2;