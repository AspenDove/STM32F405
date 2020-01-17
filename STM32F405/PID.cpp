#include "PID.h"
//PID pid[12];
//void PID::Init(PID* pid)
//{
//	//memset(pid, 0, sizeof(PID) * 12);
//	for (int i = 0; i != 4; ++i)//底盘速度环
//	{
//		pid[i].m_Kp = 2000.f / 1.e2f;
//		pid[i].m_Ti = 655.f / 1.e4f;
//		pid[i].m_Td = 349.f / 1.e6;
//	}
//	for (int i = 4; i != 6; ++i)//云台速度环
//	{
//		pid[i].m_Kp = 0.64f;//4.2f;
//		pid[i].m_Ti = 0.82f;//1.0f;
//		pid[i].m_Td = 6.4f;//10.f;
//		pid[i].m_alpha = 0.5f;
//	}
//	for (int i = 6; i != 7; ++i)//升降结构速度
//	{
//		pid[i].m_Kp = 600.f / 1.e5f;
//		pid[i].m_Ti = 4400.f / 1.e4;
//		pid[i].m_Td = 4878.f / 1.e3;
//	}
//	for (int i = 7; i != 8; ++i)//拨弹速度
//	{
//		pid[i].m_Kp = 2.f;
//		pid[i].m_Ti = 0.08f;
//		pid[i].m_Td = 20.f;
//	}
//	
//	for (int i = 8; i != 10; ++i)//云台角度
//	{
//		pid[i].m_Kp = 0.48f;//0.9f;
//		pid[i].m_Ti = 0.034f;//0.0f;
//		pid[i].m_Td = 0.9f;//1.6f;
//		pid[i].m_alpha = 0.1f;
//	}
//	for (int i = 10; i != 11; ++i)//升降结构角度
//	{
//		pid[i].m_Kp = 4378.f / 1.e4f;
//		pid[i].m_Ti = 0.f;
//		pid[i].m_Td = 4411.f / 1.e3;
//	}
//	for (int i = 11; i != 12; ++i)//拨弹角度
//	{
//		pid[i].m_Kp = 0.036f;
//		pid[i].m_Ti = 0.f;
//		pid[i].m_Td = 0.4f;
//		pid[i].m_alpha = 0.4f;
//	}
//}
void PID::Adjust(float m_Kp, float m_Ti, float m_Td, float alpha)
{
	m_Kp = m_Kp;
	m_Ti = m_Ti;
	m_Td = m_Td;
	m_alpha = alpha;
}
float PID::Delta(const float error)
{
	const float T = 1.e-3f;
	m_error[LLAST] = m_error[LAST] * 0.92f;
	m_error[LAST] = m_error[NOW] * 0.92f;
	m_error[NOW] = error * 1.08f;
	//	float A = pid->m_Kp*(1.f + T / pid->m_Ti + pid->m_Td / T);
	//	float B = pid->m_Kp*(1.f + pid->m_Td * 2.f / T);
	//	float C = pid->m_Kp*(pid->m_Td / T);
	//	
	//	return pid->m_error[NOW]*A - pid->m_error[LAST]*B + pid->m_error[LLAST]*C;
	return m_Kp * (m_error[NOW] - m_error[LAST]) + m_Ti * m_error[NOW] + m_Td * (m_error[NOW] - 2 * m_error[LAST] + m_error[LLAST]);
	//m_Kp = 1.14809049870968f;
	//m_Ti = 0.44716208134003f;
	//float delta = 0.12f;
	//return m_Kp*((1.f + delta / m_Ti)*m_error[NOW] + (-1)*m_error[LAST]);
}
float PID::Filter(const float delta)
{
	int32_t sum = 0;
	m_filter[m_filterindex++] = delta;
	if (m_filterindex == FILTER)m_filterindex = 0;
	for (int16_t t = 0; t != FILTER; t++)
		sum += m_filter[t];
	return sum / static_cast<float>(FILTER);
}
float PID::Position(const float error)
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