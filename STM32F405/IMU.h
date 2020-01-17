#pragma once
#include <stm32f4xx_hal.h>
#include <arm_math.h>
#include "delay.h"

#define SAVE 			0x00
#define CALSW 		0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 			0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD		0x1c

#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c			
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL				0x4f
#define GPSVH				0x50

#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5		


class IMU
{
public:
	uint8_t* GetDMARx()
	{
		return rxData;
	}
	void OnIRQHandler(size_t rsize)
	{
		const uint8_t* ucRxBuffer = this->rxData;
		for (uint8_t i = 0; i != 99; ++i)
		{
			if (ucRxBuffer[i] == 0x55)
			{
				switch (ucRxBuffer[i + 1])
				{
					//case 0x50:	memcpy(&stcTime, &ucRxBuffer[i + 2], 8); break;
					//case 0x51:	memcpy(&stcAcc, &ucRxBuffer[i + 2], 8); break;
					//case 0x52:	memcpy(&stcGyro, &ucRxBuffer[i + 2], 8); break;
				case 0x53:	memcpy(&stcAngle, &ucRxBuffer[i + 2], 8); break;
					//case 0x54:	memcpy(&stcMag, &ucRxBuffer[i + 2], 8); break;
					//case 0x55:	memcpy(&stcDStatus, &ucRxBuffer[i + 2], 8); break;
					//case 0x56:	memcpy(&stcPress, &ucRxBuffer[i + 2], 8); break;
					//case 0x57:	memcpy(&stcLonLat, &ucRxBuffer[i = 2], 8); break;
					//case 0x58:	memcpy(&stcGPSV, &ucRxBuffer[i + 2], 8); break;
				default:;
				}
			}
		}
		raw_angle.roll = stcAngle.Angle[0] * 180.f / 32768.f;
		raw_angle.pitch = stcAngle.Angle[1] * 180.f / 32768.f;
		raw_angle.yaw = stcAngle.Angle[2] * 180.f / 32768.f;

		angle.roll += raw_angle.roll - pre_angle.roll;
		angle.yaw += raw_angle.yaw - pre_angle.yaw;
		angle.pitch += raw_angle.pitch - pre_angle.pitch;

		pre_angle = raw_angle;
	}

	typedef struct
	{
		float x, y, z;
	}Accel, Gyro, Mag;
	typedef struct
	{
		float pitch, roll, yaw;
	}Angle;

	struct STime
	{
		uint8_t ucYear;
		uint8_t ucMonth;
		uint8_t ucDay;
		uint8_t ucHour;
		uint8_t ucMinute;
		uint8_t ucSecond;
		uint16_t usMiliSecond;
	};
	struct SAcc
	{
		int16_t a[3];
		int16_t t;
	};
	struct SGyro
	{
		int16_t w[3];
		int16_t t;
	};
	struct SAngle
	{
		int16_t Angle[3];
		int16_t t;
	};
	struct SMag
	{
		int16_t h[3];
		int16_t t;
	};

	struct SDStatus
	{
		int16_t sDStatus[4];
	};
	struct SPress
	{
		long lPressure;
		long lAltitude;
	};
	struct SLonLat
	{
		long lLon;
		long lLat;
	};
	struct SGPSV
	{
		int16_t sGPSHeight;
		int16_t sGPSYaw;
		int32_t lGPSVelocity;
	};

	Accel accel = { 0,0,0 };
	Gyro gyro{};
	Mag mag{};
	__IO Angle angle{};
	Angle raw_angle{}, pre_angle{};
	__IO Angle &setangle = angle;
	uint8_t rxData[99];
private:
	STime stcTime;
	SAcc stcAcc;
	SGyro stcGyro;
	SAngle stcAngle;
	SMag stcMag;
	SDStatus stcDStatus;
	SPress stcPress;
	SLonLat stcLonLat;
	SGPSV stcGPSV;
};

extern IMU imu;