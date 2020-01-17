#include "delay.h"

void delay_init(const uint8_t sysclk)
{
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTickƵ��ΪHCLK
	fac_us = sysclk;						//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
}		

void delay_us(const uint32_t nus)
{
	uint32_t tcnt = 0;
	const uint32_t reload = SysTick->LOAD;				//LOAD��ֵ	    	 
	const uint32_t ticks = nus * fac_us; 						//��Ҫ�Ľ����� 
	uint32_t told = SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while (true)
	{
		const uint32_t tnow = SysTick->VAL;	
		if (tnow != told)
		{	    
			if (tnow < told)tcnt += told - tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt += reload - tnow + told;	    
			told = tnow;
			if (tcnt >= ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	}
}
/*void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD = fac_us*nus;
	SysTick->VAL = 0X00;//��ռ�����
	SysTick->CTRL = 0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
	do
	{
		temp = SysTick->CTRL;//��ȡ��ǰ������ֵ
	} while ((temp & 0x01)&&(!(temp&(1 << 16))));//�ȴ�ʱ�䵽��
	SysTick->CTRL = 0x00; //�رռ�����
	SysTick->VAL = 0X00; //��ռ�����
}*/
void start(void)
{
	SysTick->LOAD = 0xffffff;
	SysTick->VAL = 0X00;//��ռ�����
	SysTick->CTRL = 0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
}
float count(void)
{
	//SysTick->CTRL = 0x00; //�رռ�����
	//SysTick->VAL = 0X00; //��ռ�����

	return (0xffffff - SysTick->VAL)/180.f;
}
void delay_ms(const uint16_t nms)
{
	for (uint32_t i = 0; i < nms; i++) delay_us(1000);
}
