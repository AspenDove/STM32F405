#pragma once
#include "stm32f4xx_hal.h"
#include "functional"
#include "gpio.h"
enum{ BASE, PWM, IC };

class TIM
{
public:
	TIM& Init(uint32_t Mode, TIM_TypeDef* TIM, uint32_t frequency);
	void MspPostInit(reuse r) const;
	void BaseInit(std::function<void(void)> fun);
	TIM& PWMInit(uint32_t channel, float duty, reuse r);
	void PWMDuty(uint32_t channel, float duty) const;
	void ICInit(int32_t channel, uint32_t ICPolarity, reuse r);

	bool operator==(const TIM_HandleTypeDef *htim)
	{
		return (&this->htim) == htim;
	}
	TIM_HandleTypeDef htim;
	uint32_t counter;
	std::function<void(void)> Ontimer{};
};

extern TIM tasks, fraction, photogate;
