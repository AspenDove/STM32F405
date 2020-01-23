#include <stm32f4xx_hal.h>

#include "sysclk.h"
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

RC		  rc;
TIM		  tasks, fraction, photogate;
CAN       can1, can2;
IMU       imu;
LED       led1, led2;
UART	  uart2, uart6;
Delay     delay;
Control   ctrl;

int main(void)
{
	HAL_Init();
	SystemClockConfig();
	delay.Init(168);

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

	tasks.Init(BASE, TIM3, 1000).BaseInit([]()
	{
		static uint8_t can1_send[16];
		static uint8_t can2_send[16];
		HAL_StatusTypeDef status = HAL_OK;

		rc.Update();
		//ctrl.Update();
		switch (tasks.counter % 5)
		{
		case 0:
			status = can1.Transmit(0x200, can1_send + 0);
			if (status != HAL_OK)led2.on();
			else led2.off();
			break;
		case 1:
			status = can2.Transmit(0x200, can2_send + 0);
			if (status != HAL_OK)led1.on();
			else led1.off();
			break;
		case 2:
			status = can1.Transmit(0x1ff, can1_send + 8);
			if (status != HAL_OK)led2.on();
			else led2.off();
			break;
		case 3:
			status = can2.Transmit(0x1ff, can2_send + 8);
			if (status != HAL_OK)led1.on();
			else led1.off();
			break;
		default:
			for (auto &motor : can1_motor)motor.Ontimer(can1.data, can1_send);
			for (auto &motor : can2_motor)motor.Ontimer(can2.data, can2_send);
		}
	});

	fraction.Init(PWM, TIM4, 330).
		PWMInit(TIM_CHANNEL_1, 0.4f, reuse{ GPIOB, GPIO_PIN_6 }).
		PWMInit(TIM_CHANNEL_2, 0.4f, reuse{ GPIOB, GPIO_PIN_7 });

	for (;;) {}
}