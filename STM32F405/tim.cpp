#include "tim.h"
#include "can.h"
#include "IMU.h"
#include "motor.h"
#include "control.h"
#include "RC.h"
#include "LED.h"


#define PI 3.1415926f
TIM_HandleTypeDef *phtim[4];

TIM& TIM::Init(const uint32_t Mode, TIM_TypeDef* TIM, const uint32_t frequency)
{
	const float x = 84000000 / frequency;
	if (x >= 5000)
	{
		htim.Init.Prescaler = x / 5000 - 1u;
		htim.Init.Period = 5000 - 1u;
	}
	else
	{
		htim.Init.Prescaler = 84u - 1u;
		htim.Init.Period = x / 84 - 1u;
	}
	htim.Instance = TIM;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	switch (Mode)
	{
	case BASE:HAL_TIM_Base_Init(&htim); break;
	case PWM:HAL_TIM_PWM_Init(&htim); break;
	case IC:HAL_TIM_IC_Init(&htim); break;
	default:;
	}
	if (TIM == TIM2)phtim[1] = &htim;
	if (TIM == TIM3)phtim[2] = &htim;
	if (TIM == TIM4)phtim[3] = &htim;
	return *this;
}
void TIM::MspPostInit(const reuse r) const
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_CLK_ENABLE(r.GPIOx);

	GPIO_InitStruct.Pin = r.Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = this->htim.Instance == TIM2 ? GPIO_AF1_TIM2 : GPIO_AF2_TIM3;
	HAL_GPIO_Init(r.GPIOx, &GPIO_InitStruct);
}

extern "C" void TIM2_IRQHandler(void)
{
	if (phtim[1])HAL_TIM_IRQHandler(phtim[1]);
}
extern "C" void TIM3_IRQHandler(void)
{
	if (phtim[2])HAL_TIM_IRQHandler(phtim[2]);
}
extern "C" void TIM4_IRQHandler(void)
{
	if (phtim[3])HAL_TIM_IRQHandler(phtim[3]);
}

void TIM::BaseInit(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);
	HAL_TIM_Base_Start_IT(&htim);
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	SET_BIT(RCC->APB1ENR, 0x1U << (((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE)) / 0x400U));
	/* Delay after an RCC peripheral clock enabling */
	__IO uint32_t tmpreg = READ_BIT(RCC->APB1ENR, 0x1U << (((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE)) / 0x400U));
	UNUSED(tmpreg);
	HAL_NVIC_SetPriority(static_cast<IRQn_Type>((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE) / 0x400U + 28), 0, 0);
	HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE) / 0x400U + 28));
}


extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t can1_send[16];
	static uint8_t can2_send[16];
	HAL_StatusTypeDef status = HAL_OK;


	if (tasks.counter++, tasks == htim)
	{
		rc.Update();
		//ctrl.Update();
		if (tasks.counter % 500 == 0)
		{
			/*
			sprintf(info, "%.2lf %d %d\r\n",time,can1_motor[0].current,can1_motor[0].curspeed);
			uart7.UARTTransmit((uint8_t*)(info), strlen(info));
			float delta = 0.01f;
			time += delta;
			if(time <= 7 * 3)
			{
				can1_motor[0].current = currentsqe[(uint32_t)(time / 3.f)];
			}
			*/
			/*i1 += (float)(int32_t)getword(can1_data[0][4], can1_data[0][5])*(float)(int32_t)getword(can1_data[0][2], can1_data[0][3]);
			i2 += judgement.data.powerHeatData.chassisPower;
			datapack data = {
				i1,
				i1/i2,
				judgement.data.powerHeatData.chassisPower };
			uart7.UARTTransmit((uint8_t*)(&data), sizeof(float) * 3);*/
		}
		if (tasks.counter % 4 == 0)
		{
			for (auto &motor : can1_motor)motor.Ontimer(can1.data, can1_send);
			for (auto &motor : can2_motor)motor.Ontimer(can2.data, can2_send);
		}
		switch (tasks.counter % 4)
		{
		case 0:
			status = can1.Transmit(0x200, can1_send + 0);
			if (status == HAL_TIMEOUT)led2.on();
			else led2.off();
			break;
		case 1:
			status = can2.Transmit(0x200, can2_send + 0);
			if (status == HAL_TIMEOUT)led1.on();
			else led1.off();
			break;
		case 2:
			status = can1.Transmit(0x1ff, can1_send + 8);
			if (status == HAL_TIMEOUT)led2.on();
			else led2.off();
			break;
		case 3:
			status = can2.Transmit(0x1ff, can2_send + 8);
			if (status == HAL_TIMEOUT)led1.on();
			else led1.off();
			break;
		default:;
		}

	}
	else if (htim->Instance == TIM4)
	{
	}
}


TIM& TIM::PWMInit(uint32_t channel, float duty, reuse r)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = static_cast<unsigned>(duty * (htim.Init.Period + 1)) - 1u;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel);

	this->MspPostInit(r);
	HAL_TIM_PWM_Start(&htim, channel);
	return *this;
}
void TIM::PWMDuty(const uint32_t channel, const float duty) const
{
	*reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(htim.Instance->CCR1) + (channel - TIM_CHANNEL_1)) = (htim.Init.Period + 1)*duty - 1u;
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
	__IO uint32_t tmpreg = 0x00U;
	SET_BIT(RCC->APB1ENR, 0x1U << (((reinterpret_cast<unsigned>(tim_pwmHandle->Instance) - APB1PERIPH_BASE)) / 0x400U));
	/* Delay after an RCC peripheral clock enabling */
	tmpreg = READ_BIT(RCC->APB1ENR, 0x1U << ((reinterpret_cast<unsigned>(tim_pwmHandle->Instance - APB1PERIPH_BASE)) / 0x400U));
	UNUSED(tmpreg);
}


void TIM::ICInit(int32_t channel, uint32_t ICPolarity, reuse r)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;
	this->MspPostInit(r);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

	sConfigIC.ICPolarity = ICPolarity;//TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	HAL_TIM_IC_ConfigChannel(&htim, &sConfigIC, channel);
	HAL_TIM_IC_Start_IT(&htim, channel);
}
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{
	SET_BIT(RCC->APB1ENR, 0x1U << ((reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE)) / 0x400U));
	/* Delay after an RCC peripheral clock enabling */
	__IO uint32_t tmpreg = READ_BIT(RCC->APB1ENR, 0x1U << ((reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE)) / 0x400U));
	UNUSED(tmpreg);
	HAL_NVIC_SetPriority(static_cast<IRQn_Type>(reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE) / 0x400U + 28), 0, 0);
	HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>(reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE) / 0x400U + 28));
}
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
			if (!(photogate.counter++ & 1))
				;
			TIM_MasterConfigTypeDef sMasterConfig;
			TIM_IC_InitTypeDef sConfigIC;
			sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
			sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
			HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig);

			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
			sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
			sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
			sConfigIC.ICFilter = 0;
			HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, htim->Channel);
			HAL_TIM_IC_Start_IT(htim, htim->Channel);
		}
	}
}

