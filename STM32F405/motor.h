#pragma once
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <cmath>
#include "PID.h"
constexpr auto MAXSPEED    = 3000;
constexpr auto ADJUSTSPEED = 3000;

enum { ID1 = 0x201, ID2, ID3, ID4, ID5, ID6, ID7, ID8 };
enum { pre = 0, now };
enum pid_mode{ speed = 0, position };
enum motor_type{ M3508, M3510, M2310, EC60, M6623, M6020 };
enum motor_mode { SPD, POS, ACE };

#define SQRTF(x) ((x)>0?sqrtf(x):-sqrtf(-x))			
#define T 1.e-3f

class Motor
{
	typedef motor_type type_t;
public:
	uint32_t ID;

	Motor(const type_t type,const motor_mode mode,const uint32_t id, PID _speed, PID _position)
		: ID(id)
		, type(type)
		, mode(mode)
	{
		getmax(type);
		memcpy(&pid[speed], &_speed, sizeof(PID));
		memcpy(&pid[position], &_position, sizeof(PID));
	}
	Motor(const type_t type, const motor_mode mode, const uint32_t id, PID _speed)
		: ID(id)
		, type(type)
		, mode(mode)
	{
		getmax(type);
		memcpy(&pid[speed], &_speed, sizeof(PID));
	}
	void Ontimer(uint8_t idata[][8], uint8_t* odata)
	{
		const uint32_t ID = this->ID - ID1;
		angle[now] = getword(idata[ID][0], idata[ID][1]);

		if(type == EC60)curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
		else curspeed = getword(idata[ID][2], idata[ID][3]);
		///angle[pre]&angle[now]&curspeed are available.
		if(mode == ACE)//accurate mode
		{
			curcircle += getdeltaa(angle[now] - angle[pre]);
			if (spinning)
			{
				const int32_t diff = curcircle - setcircle;
				if (std::fabs(diff) >= 4096) setspeed = -ADJUSTSPEED * SQRTF((float)diff / (float)spinning);
				else setangle = std::fabs(diff), spinning = 0;
			}
			else
			{
				const float error = getdeltaa(setangle - angle[now]);
				if (std::fabs(error) < 50.f)setspeed = 0.0;
				else
				{
					setspeed = pid[position].Position(pid[position].Filter(
						getdeltaa(setangle - angle[now])));
					setspeed = setrange(setspeed, maxspeed);
				}
			}
			current += pid[speed].Delta(setspeed - curspeed);
			current = setrange(current, 13000);
		}
		else if(mode == POS)//position mode
		{
			const float error = getdeltaa(setangle - angle[now]);
			//if (fabs(error) < 50.f)setspeed = 0.0;
			//else
			{
				setspeed = pid[position].Position((
					getdeltaa(setangle - angle[now])));
				setspeed = setrange(setspeed, maxspeed);
			}
			current = pid[speed].Position(setspeed - curspeed);
			current = setrange(current, maxcurrent);
		}
		else if(mode == SPD)//speed mode
		//proceed speed any way.
		{
			current += pid[speed].Delta(setspeed - curspeed);
			current = setrange(current, maxcurrent);
		}

		angle[pre] = angle[now];
		odata[ID * 2] = (current & 0xff00) >> 8;
		odata[ID * 2 + 1] = current & 0x00ff;
	}
private:
	void getmax(const type_t type)
	{
		switch(type)
		{
		case M3508:
		case M3510:
			maxcurrent = 13000;
			maxspeed = 9000;
			break;
		case M2310:
			maxcurrent = 13000;
			maxspeed = 9000;
			break;
		case EC60:
			maxcurrent = 5000;
			maxspeed = 300;
			break;
		case M6623:
			maxcurrent = 5000;
			maxspeed = 300;
			break;
		case M6020:
			maxcurrent = 30000;
			maxspeed = 300;
			break;
		default:;
		}
	}

	static int16_t getdeltaa(int16_t diff)
	{
		if (diff <= -4096)
			diff += 8192;	
		else if (diff > 4096)
			diff -= 8192;
		return diff;
	}

	static int16_t getword(const uint8_t high, const uint8_t low)
	{
		const int16_t word = high;
		return (word << 8) + low;
	}

	static int16_t setrange(const int16_t original, const int16_t range)
	{
		return fmaxf(fminf(range, original), -range);
	}
	type_t type;
public:
	int16_t current{}, curspeed{}, setspeed{};
	int16_t maxspeed{}, maxcurrent{};
	int16_t setangle{}, angle[2]{};
	int32_t curcircle{}, spinning{}, setcircle{};
	int32_t mode{};
	PID pid[2];
};

extern Motor can1_motor[5];
extern Motor can2_motor[4];
