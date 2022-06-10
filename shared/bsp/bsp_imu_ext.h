/****************************************************************************
*                                                                          *
*  Copyright (C) 2022 RoboMaster.                                          *
*  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
*                                                                          *
*  This program is free software: you can redistribute it and/or modify    *
*  it under the terms of the GNU General Public License as published by    *
*  the Free Software Foundation, either version 3 of the License, or       *
*  (at your option) any later version.                                     *
*                                                                          *
*  This program is distributed in the hope that it will be useful,         *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
*  GNU General Public License for more details.                            *
*                                                                          *
*  You should have received a copy of the GNU General Public License       *
*  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
*                                                                          *
****************************************************************************/

#pragma once

#include "i2c.h"
#include "arm_math.h"

namespace bsp {

const uint16_t YYMM       = 0x30;
const uint16_t DDHH       = 0x31;
const uint16_t MMSS       = 0x32;
const uint16_t MS         = 0x33;
const uint16_t AX         = 0x34;
const uint16_t AY         = 0x35;
const uint16_t AZ         = 0x36;
const uint16_t GX         = 0x37;
const uint16_t GY         = 0x38;
const uint16_t GZ         = 0x39;
const uint16_t HX         = 0x3a;
const uint16_t HY         = 0x3b;
const uint16_t HZ         = 0x3c;
const uint16_t Roll       = 0x3d;
const uint16_t Pitch      = 0x3e;
const uint16_t Yaw        = 0x3f;
const uint16_t TEMP       = 0x40;
const uint16_t D0Status   = 0x41;
const uint16_t D1Status   = 0x42;
const uint16_t D2Status   = 0x43;
const uint16_t D3Status   = 0x44;
const uint16_t PressureL  = 0x45;
const uint16_t PressureH  = 0x46;
const uint16_t HeightL    = 0x47;
const uint16_t HeightH    = 0x48;
const uint16_t LonL       = 0x49;
const uint16_t LonH       = 0x4a;
const uint16_t LatL       = 0x4b;
const uint16_t LatH       = 0x4c;
const uint16_t GPSHeight  = 0x4d;
const uint16_t GPSYAW     = 0x4e;
const uint16_t GPSVL      = 0x4f;
const uint16_t GPSVH      = 0x50;
const uint16_t Q0         = 0x51;
const uint16_t Q1         = 0x52;
const uint16_t Q2         = 0x53;
const uint16_t Q3         = 0x54;

typedef struct {
  unsigned char ucYear;
  unsigned char ucMonth;
  unsigned char ucDay;
  unsigned char ucHour;
  unsigned char ucMinute;
  unsigned char ucSecond;
  unsigned short usMiliSecond;
} __packed Time;

typedef struct {
  short a[3];
  short T;
} __packed Acc;

typedef struct {
  short w[3];
  short T;
} __packed Gyro;

typedef struct {
  short Angle[3];
  short T;
} __packed Angle;

typedef struct {
  short h[3];
  short T;
} __packed Mag;

typedef struct {
  short sDStatus[4];
} __packed DStatus;

typedef struct {
  long lPressure;
  long lAltitude;
} __packed Press;

typedef struct {
  long lLon;
  long lLat;
} __packed LonLat;

typedef struct {
  short sGPSHeight;
  short sGPSYaw;
  long lGPSVelocity;
} __packed GPSV;

typedef struct {
  short Q[4];
} __packed Quaternion;

class WT901 {
 public:
  WT901(I2C_HandleTypeDef* i2c, uint16_t DevAddr);

  bool IsReady();
  bool GetAngle(float *angle);
  bool GetQuaternion(float *Q);
  bool GetAcc(float *acc);
  bool GetGyro(float *gyro);
  bool GetMag(int *mag);
  bool SetAngleOffset();
 
 private:
  bool GetRawAngle_(float *angle);
  I2C_HandleTypeDef* i2c_;
  uint16_t DevAddr_;
  float angle_offset_[3];
};

}
