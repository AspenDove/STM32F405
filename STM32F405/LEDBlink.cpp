#include <stm32f4xx_hal.h>

#include "can.h"
#include "tim.h"
#include "PID.h"
#include "gpio.h"
#include "usart.h"
#include "delay.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "IMU.h"
#include "LED.h"


#ifdef __cplusplus
extern "C"
#endif
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
Motor can1_motor[5] = {
	Motor(M3508,SPD, ID1, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508,SPD, ID2, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508,SPD, ID3, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508,SPD, ID4, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M6020,POS, ID5, PID(18.0f, 1.5f, 40.f,0.6f), PID(0.3f, 0.001f, 40.f, 0.75f)),
};

Motor can2_motor[4] = {
	Motor(M3508,SPD, ID1, PID(3.f, 0.0655f, 3.49e-4f)),
	Motor(M3508,SPD, ID2, PID(3.f, 0.0655f, 3.49e-4f)),
	Motor(M2310,ACE, ID3, PID(4.0f, 0.1f, 5.f), PID(0.036f, 0.f, 0.4f, 0.4f)),
	Motor(M6020,POS, ID6, PID(18.0f, 1.5f, 40.f,0.6f), PID(0.3f, 0.001f, 40.f, 0.75f))
};

TIM		  tasks, fraction, photogate;
CAN       can1, can2;
IMU       imu;
RC		  rc;
UART	  uart2, uart6;
Control   ctrl;
LED       led1,led2;

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	}

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
	}

	/**Configure the Systick interrupt time
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	delay_init(168);

	led1.Init({ GPIOC,GPIO_PIN_13 });
	led2.Init({ GPIOC,GPIO_PIN_14 });

	uart2.Init(USART2, 100000).DMARxInit(rc.GetDMARx());
//	uart6.Init(USART6, 115200).DMATxInit().DMARxInit(imu.GetDMARx());
	//uart7.Init(UART7, 115200).DMATxInit().DMARxInit(imu.GetDMARx(), 99);
	//uart8.Init(UART8, 115200).DMATxInit().DMARxInit(nuc.GetDMARx());

	//imu.Init();

	ctrl.Init(
		{ &can1_motor[0],&can1_motor[1],&can1_motor[2],&can1_motor[3] },
		{ &can1_motor[4],&can2_motor[3] }, &imu,
		{ &can2_motor[0],&can2_motor[1],&can2_motor[2] });

	can1.Init(CAN1);
	can2.Init(CAN2);

	tasks.Init(BASE, TIM3, 1000).BaseInit();

	fraction.Init(PWM, TIM4, 800).
		PWMInit(TIM_CHANNEL_1, 0.4f, reuse{ GPIOB, GPIO_PIN_6 }).
		PWMInit(TIM_CHANNEL_2, 0.4f, reuse{ GPIOB, GPIO_PIN_7 });

	for (;;){}
}