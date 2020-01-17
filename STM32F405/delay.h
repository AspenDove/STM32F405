#pragma once
#include "stm32f4xx_hal.h"

static uint32_t fac_us=0;							//usÑÓÊ±±¶³ËÊı
void delay_init(uint8_t sysclk);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void start(void);
float count(void);

