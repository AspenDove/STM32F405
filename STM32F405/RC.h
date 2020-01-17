#pragma once
#include "usart.h"
#include "control.h"


#define W     ((uint8_t)0x01<<0)
#define S     ((uint8_t)0x01<<1)
#define A     ((uint8_t)0x01<<2)
#define D     ((uint8_t)0x01<<3)
#define SHIFT ((uint8_t)0x01<<4)
#define CTRL  ((uint8_t)0x01<<5)
#define Q     ((uint8_t)0x01<<6)
#define E     ((uint8_t)0x01<<7)

#define R     ((uint8_t)0x01<<0)
#define F     ((uint8_t)0x01<<1)
#define G     ((uint8_t)0x01<<2)
#define Z     ((uint8_t)0x01<<3)
#define X     ((uint8_t)0x01<<4)
#define C     ((uint8_t)0x01<<5)

class RC
{
public:
	struct
	{
		int16_t ch[4];
		uint8_t s[2];
	}rc;
	struct
	{
		int16_t x, y, z;
		uint8_t press_l, press_r, key_h, key_l;
		const float spdratio = 0.05f;
	}pc;

	uint8_t* GetDMARx(void) { return m_frame; }

	void OnIRQHandler(size_t rxSize)
	{
		if (rxSize != 18)return;
		if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

		rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
		rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
		rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
		rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
		if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
		if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
		if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
		if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

		rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
		rc.s[1] = ((m_frame[5] >> 4) & 0x03);

		pc.x = m_frame[6] | (m_frame[7] << 8);
		pc.y = m_frame[8] | (m_frame[9] << 8);
		pc.z = m_frame[10] | (m_frame[11] << 8);
		pc.press_l = m_frame[12];
		pc.press_r = m_frame[13];

		pc.key_h = m_frame[15];
		pc.key_l = m_frame[14];
	}

	void manual_pantile()
	{
		const float adjangle = 3.f;
		if (rc.ch[3] < 0)ctrl.pantile.setpitch += adjangle;
		else if (rc.ch[3] > 0)ctrl.pantile.setpitch -= adjangle;
		if (rc.ch[2] < 0)ctrl.pantile.setyaw += adjangle;
		else if (rc.ch[2] > 0)ctrl.pantile.setyaw -= adjangle;
	}
	void OnRC()
	{
		ctrl.shooter.shoot = (rc.s[1] == 2);
		if (rc.s[1] == 1)
		{
			ctrl.pantile.follow = true;
			ctrl.direction_speed(
				rc.ch[1] * MAXSPEED / 660,
				rc.ch[0] * MAXSPEED / 660);
			manual_pantile();
			return;
		}
		ctrl.pantile.aim = (rc.s[0] == 2);
		ctrl.pantile.follow = (rc.s[0] == 1);
		if (rc.s[0] == 1)
		{
			ctrl.follow_speed(
				rc.ch[1] * MAXSPEED / 660,
				rc.ch[0] * MAXSPEED / 660);
			ctrl.chassis.speedz = rc.ch[2] * MAXSPEED / 660;
			return;
		}
		if (rc.s[0] == 3)
		{
			ctrl.follow_speed(
				rc.ch[1] * MAXSPEED / 660,
				rc.ch[0] * MAXSPEED / 660);
			manual_pantile();
			return;
		}
		if (rc.s[0] == 2)
		{
			manual_pantile();
			ctrl.chassis.speedx = rc.ch[1] * MAXSPEED / 660;
			ctrl.chassis.speedy = rc.ch[0] * MAXSPEED / 660;
			return;
		}
	}
	void OnPC()
	{
		const float adjspeed = MAXSPEED  / (1+sqrtf(2.f));
		ctrl.chassis.speedx = ctrl.chassis.speedy = ctrl.chassis.speedz = 0;
		ctrl.direction_speed(
			((int)!!(pc.key_l & W)*(1) + (int)!!(pc.key_l & S)*(-1))*adjspeed,
			((int)!!(pc.key_l & D)*(1) + (int)!!(pc.key_l & A)*(-1))*adjspeed);
		if (pc.key_l & Q)ctrl.chassis.speedz -= adjspeed;
		if (pc.key_l & E)ctrl.chassis.speedz += adjspeed;

		ctrl.pantile.setyaw   -= pc.spdratio * pc.x;
		ctrl.pantile.setpitch += pc.spdratio * pc.y;

		ctrl.shooter.shoot = pc.press_l;
	}

	void Update()
	{
		OnRC();
		//OnPC();
	}
private:
	uint8_t m_frame[UART_MAX_LEN]{};
};

extern RC rc;
