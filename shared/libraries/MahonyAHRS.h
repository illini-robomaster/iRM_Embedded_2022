///****************************************************************************
//*                                                                          *
//*  Copyright (C) 2022 RoboMaster.                                          *
//*  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
//*                                                                          *
//*  This program is free software: you can redistribute it and/or modify    *
//*  it under the terms of the GNU General Public License as published by    *
//*  the Free Software Foundation, either version 3 of the License, or       *
//*  (at your option) any later version.                                     *
//*                                                                          *
//*  This program is distributed in the hope that it will be useful,         *
//*  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
//*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
//*  GNU General Public License for more details.                            *
//*                                                                          *
//*  You should have received a copy of the GNU General Public License       *
//*  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
//*                                                                          *
//****************************************************************************/
//
//#pragma once
//
//#include "bsp_imu.h"
//
//namespace control {
//
//class MahonyAHRS {
// public:
//  MahonyAHRS(bsp::MPU6500* imu, float Ki = 0.01f, float Kp = 2.0f);
//  void MPUMeasureOffset(int cmd);
//  void InitQuaternion();
//  void Update();
//  volatile float pitch, yaw, roll;
// private:
//  float InvSqrt(float x);
//  bsp::MPU6500* imu;
//  float Kp, Ki;
//
//  int32_t last_update;
//  volatile float q0, q1, q2, q3;
//  volatile float exInt, eyInt, ezInt;
//  volatile float gx, gy, gz;
//  volatile float ax, ay, az;
//  volatile float mx, my, mz;
//
//  volatile float ax_offset, ay_offset, az_offset;
//  volatile float gx_offset, gy_offset, gz_offset;
//};
//
//}
