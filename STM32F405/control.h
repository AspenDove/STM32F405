#pragma once
#include <stm32f4xx_hal.h>
#include <array>
#include "imu.h"
#include "motor.h"
#include "tim.h"
#include <cmath>

class Control
{
public:
	struct Chassis
	{
		Motor *chassis[4];
		int16_t speedx{}, speedy{}, speedz{};
		float bias = sqrt(2);
		void Update()
		{
			chassis[0]->setspeed = +speedy * bias + speedx + speedz;
			chassis[1]->setspeed = +speedy * bias - speedx + speedz;
			chassis[2]->setspeed = -speedy * bias + speedx + speedz;
			chassis[3]->setspeed = -speedy * bias - speedx + speedz;
		}
	};
	struct Pantile
	{
		const int16_t midyaw = 4096;
		IMU     *imu;
		Motor   *pantile[2];
		float setpitch = 4200, setyaw = midyaw;

		//int16_t speedpitch, speedyaw  ;
		int16_t pitchmax = 4824, pitchmin = 3579;
		bool follow = true;
		bool pre_follow = false;

		bool aim = false;
		PID pid[2] =
		{
			PID(20,0.5,100),
			PID(20,0.5,100)
		};
		void Update()
		{
			if (setyaw >= 8192)setyaw -= 8192;
			if (setyaw <= 0)setyaw += 8192;

			if (setpitch <= pitchmin)setpitch = pitchmin;
			if (setpitch >= pitchmax)setpitch = pitchmax;
			int16_t anglepitch = setpitch, angleyaw = setyaw;

			if (follow && !pre_follow)
			{
				imu->setangle.yaw = 360.f*pantile[0]->angle[now] / 8192.f - 180.f;//8192.f*(imu->angle.yaw     + 180.f) / 360.f;
				imu->setangle.pitch = 360.f*pantile[1]->angle[now] / 8192.f - 180.f;//setpitch = 8192.f*(imu->angle.pitch + 180.f) / 360.f;
			}
			if (follow)
			{
				const float jy_yaw = 8192.f*(imu->angle.yaw + 180.f) / 360.f;
				const float jy_pitch = 8192.f*(imu->angle.pitch + 180.f) / 360.f;
				if (tasks.counter >= 2000)
				{
					angleyaw = pantile[0]->angle[now] + setyaw - jy_yaw;
					anglepitch = pantile[1]->angle[now] + setpitch - jy_pitch;
					pre_follow = true;
				}
			}
			else
			{
				pre_follow = false;
				anglepitch = setpitch;
				angleyaw = setyaw;
			}
			if (angleyaw >= 8192)angleyaw -= 8192;
			if (angleyaw <= 0)angleyaw += 8192;

			if (anglepitch <= pitchmin)anglepitch = pitchmin;
			if (anglepitch >= pitchmax)anglepitch = pitchmax;

			pantile[0]->setangle = angleyaw;
			pantile[1]->setangle = anglepitch;
		}
	};
	struct Shooter
	{
		Motor *shooter[3];
		int16_t freq = 2, speed = 6000;
		bool shoot = false;
		void Update()
		{
			if (shoot)
			{
				counter++;
				shooter[0]->setspeed = -speed;
				shooter[1]->setspeed = speed;

				if (counter % (1000 / freq) == 0)
				{
					shooter[2]->spinning = 36 * 8192 / 9;
					shooter[2]->setcircle += shooter[2]->spinning;
				}
			}
			else
			{
				shooter[0]->setspeed = 0;
				shooter[1]->setspeed = 0;
				counter = 0;
			}
		}
	private:
		uint32_t counter = 0;
	};

	Chassis chassis{};
	Pantile pantile{};
	Shooter shooter{};

	void follow_speed(const int16_t speedx, const int16_t speedy)
	{
		float theta = 2 * PI*(pantile.pantile[0]->angle[now] - pantile.midyaw) / 8192.f;
		float st = sin(theta);
		float ct = cos(theta);

		chassis.speedy = -speedx * st + speedy * ct;
		chassis.speedx = +speedx * ct + speedy * st;
	}
	void direction_speed(const int16_t speedx, const int16_t speedy)
	{
		if (speedx != 0 || speedy != 0)
		{
			float theta = (pantile.pantile[0]->angle[now] - pantile.midyaw);
			theta = std::min(std::max(theta*2 , -2000.f), 2000.f);
			chassis.speedz = -theta;
		}
		else chassis.speedz = 0;

		chassis.speedx = speedx;
		chassis.speedy = speedy;
	}

	void Init(
		std::array<Motor*, 4> chassis,
		std::array<Motor*, 2> pantile, IMU* imu,
		std::array<Motor*, 3> shooter)
	{
		for (size_t i = 0; i != 4; ++i)
			this->chassis.chassis[i] = chassis[i];
		for (size_t i = 0; i != 2; ++i)
			this->pantile.pantile[i] = pantile[i];
		this->pantile.imu = imu;
		for (size_t i = 0; i != 3; ++i)
			this->shooter.shooter[i] = shooter[i];
	}
	//call me at 1khz
	void Update()
	{
		chassis.Update();
		pantile.Update();
		shooter.Update();
	}
};

extern Control ctrl;