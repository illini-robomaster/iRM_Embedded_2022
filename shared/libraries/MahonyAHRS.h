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
