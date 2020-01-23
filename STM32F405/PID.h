#pragma once
#include "stm32f4xx_hal.h"

enum { INTEGRATE = 0, LLAST = 0, LAST = 1, NOW = 2 };
#define FILTER 2
class PID
{
public:
	PID(void)
		: m_Kp(0.f)
		, m_Ti(0.f)
		, m_Td(0.f), m_alpha(0.f) {}
	PID(float Kp, float Ti, float Td, float alpha = 0.0f)
		: m_Kp(Kp)
		, m_Ti(Ti)
		, m_Td(Td), m_alpha(alpha) {}

	void Adjust(float Kp, float Ti, float Td, float alpha)
	{
		m_Kp = Kp;
		m_Ti = Ti;
		m_Td = Td;
		m_alpha = alpha;
	}
	float Filter(float delta)
	{
		int32_t sum = 0;
		m_filter[m_filterindex++] = delta;
		if (m_filterindex == FILTER)m_filterindex = 0;
		for (int16_t t = 0; t != FILTER; t++)
			sum += m_filter[t];
		return sum / static_cast<float>(FILTER);
	}
	float Delta(float error)
	{
		m_error[LLAST] = m_error[LAST] * 0.92f;
		m_error[LAST] = m_error[NOW] * 0.92f;
		m_error[NOW] = error * 1.08f;

		return m_Kp * (m_error[NOW] - m_error[LAST]) + m_Ti * m_error[NOW] + m_Td * (m_error[NOW] - 2 * m_error[LAST] + m_error[LLAST]);
		//m_Kp = 1.14809049870968f;
		//m_Ti = 0.44716208134003f;
		//float delta = 0.12f;
		//return m_Kp*((1.f + delta / m_Ti)*m_error[NOW] + (-1)*m_error[LAST]);
	}
	float Position(float error)
	{
		m_error[NOW] = error;// *1.08f;
		//float beta = 1.f;
		//if (fabs(m_error[NOW]) > 800.f)beta = 0.f;//积分分离
		if (!((m_error[INTEGRATE] >= 5000.f&&m_error[NOW] > 0.f) ||
			(m_error[INTEGRATE] <= -5000.f&&m_error[NOW] < 0.f)))
			m_error[INTEGRATE] += m_error[NOW];
		//不完全微分
		this->m_lderivative = m_Td * (1.f - m_alpha)*(m_error[NOW] - m_error[LAST]) + m_alpha * m_lderivative;
		const float result = this->m_error[NOW] * this->m_Kp + this->m_error[INTEGRATE] * this->m_Ti + this->m_lderivative;
		m_error[LAST] = m_error[NOW];//* 0.92f;
		return result;
	}

	float m_Kp, m_Ti, m_Td;
	float m_error[3] = { 0 };
private:
	float m_alpha = 0.f, m_lderivative = 0.f;
	int16_t m_filter[FILTER] = { 0 };
	uint16_t m_filterindex = 0;
};
