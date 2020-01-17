#include "delay.h"

void delay_init(const uint8_t sysclk)
{
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTick频率为HCLK
	fac_us = sysclk;						//不论是否使用OS,fac_us都需要使用
}		

void delay_us(const uint32_t nus)
{
	uint32_t tcnt = 0;
	const uint32_t reload = SysTick->LOAD;				//LOAD的值	    	 
	const uint32_t ticks = nus * fac_us; 						//需要的节拍数 
	uint32_t told = SysTick->VAL;        				//刚进入时的计数器值
	while (true)
	{
		const uint32_t tnow = SysTick->VAL;	
		if (tnow != told)
		{	    
			if (tnow < told)tcnt += told - tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt += reload - tnow + told;	    
			told = tnow;
			if (tcnt >= ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	}
}
/*void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD = fac_us*nus;
	SysTick->VAL = 0X00;//清空计数器
	SysTick->CTRL = 0X01;//使能，减到零是无动作，采用外部时钟源
	do
	{
		temp = SysTick->CTRL;//读取当前倒计数值
	} while ((temp & 0x01)&&(!(temp&(1 << 16))));//等待时间到达
	SysTick->CTRL = 0x00; //关闭计数器
	SysTick->VAL = 0X00; //清空计数器
}*/
void start(void)
{
	SysTick->LOAD = 0xffffff;
	SysTick->VAL = 0X00;//清空计数器
	SysTick->CTRL = 0X01;//使能，减到零是无动作，采用外部时钟源
}
float count(void)
{
	//SysTick->CTRL = 0x00; //关闭计数器
	//SysTick->VAL = 0X00; //清空计数器

	return (0xffffff - SysTick->VAL)/180.f;
}
void delay_ms(const uint16_t nms)
{
	for (uint32_t i = 0; i < nms; i++) delay_us(1000);
}
