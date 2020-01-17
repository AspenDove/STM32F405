#pragma once
#include "stm32f4xx_hal.h"

enum{ INTEGRATE = 0, LLAST = 0, LAST = 1, NOW = 2 };
#define FILTER 2
class PID
{
public:
	PID(void)
		: m_Kp(0.f)
		, m_Ti(0.f)
		, m_Td(0.f),m_alpha(0.f) {}
	PID(float Kp, float Ti, float Td,float alpha = 0.0f)
		: m_Kp(Kp)
		, m_Ti(Ti)
		, m_Td(Td),m_alpha(alpha) {}
	void Init(PID* pid);
	void Adjust(float Kp, float Ti, float Td,float alpha);
	float Filter(float delta);
	float Delta(float error);
	float Position(float error);
	
	float m_Kp, m_Ti, m_Td;
	float m_error[3] = { 0 };
private:
	float m_alpha = 0.f, m_lderivative = 0.f;
	int16_t m_filter[FILTER] = { 0 };
	uint16_t m_filterindex = 0;
};
