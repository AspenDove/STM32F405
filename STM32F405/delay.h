#pragma once
#include "stm32f4xx_hal.h"

class Delay
{
public:
	void Init(const uint32_t sysclk)
	{
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
		fac_us = sysclk;
	}
	void us(const uint32_t nus)
	{
		uint32_t tcnt = 0;
		const uint32_t reload = SysTick->LOAD;
		const uint32_t ticks = nus * fac_us;
		uint32_t told = SysTick->VAL;
		while (true)
		{
			const uint32_t tnow = SysTick->VAL;
			if (tnow != told)
			{
				if (tnow < told)tcnt += told - tnow;
				else tcnt += reload - tnow + told;
				told = tnow;
				if (tcnt >= ticks)break;
			}
		}
	}
	void ms(const uint32_t nms)
	{
		for (uint32_t i = 0; i < nms; i++) us(1000);
	}

	uint32_t fac_us;
};