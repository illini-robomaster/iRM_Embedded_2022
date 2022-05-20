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

#include "pose.h"

#include <cmath>

#include "arm_math.h"
#include "bsp_imu.h"
#include "cmsis_os.h"
#include "utils.h"

// Factor from us to s
static const float USEC_TO_SEC = 1000000.0;
static const uint8_t X = 0;
static const uint8_t Y = 1;
static const uint8_t Z = 2;
static const uint8_t X_NEG = 3;
static const uint8_t Y_NEG = 4;
static const uint8_t Z_NEG = 5;

namespace control {

Pose::Pose(bsp::MPU6500* _imu) : imu(_imu) {
  if (_imu == nullptr) {
    RM_ASSERT_TRUE(false, "invalid imu");
  }
  gravityDir = Z;

  // Set weight for gyro data in complementary filter to 0.98. This is an arbitrary design.
  SetAlpha(0.98);
  // Set all IMU offsets to 0
  SetOffset(0, 0, 0, 0, 0, 0);
  // Reset roll, pitch and timestamp.
  PoseInit();
}

void Pose::PoseInit(void) {
  timestamp = imu->timestamp;
  roll = 0;
  pitch = 0;
  yaw = 0;
}

void Pose::Calibrate(void) {
  // measure 100 times to compute the average
  Calibrate(100);
}

void Pose::Calibrate(int16_t _num) {
  float acce_x = 0;
  float acce_y = 0;
  float acce_z = 0;
  float gyro_x = 0;
  float gyro_y = 0;
  float gyro_z = 0;

  // average NUM times acc and gyro reads
  for (int i = 0; i < _num; i++) {
    acce_x += imu->acce.x;
    acce_y += imu->acce.y;
    acce_z += imu->acce.z;
    gyro_x += imu->gyro.x;
    gyro_y += imu->gyro.y;
    gyro_z += imu->gyro.z;
    osDelay(10);
  }

  acc_x_off = acce_x / (float)_num;
  acc_y_off = acce_y / (float)_num;
  acc_z_off = acce_z / (float)_num;

  /** Explanation: For calibration, we want the mean of noise distribution (bias)
   *  to be zeroed when IMU is placed on a flat plane.
   *  Note that acc_z_off should have a constant gravity offset on a flat plane,
   *  which also has a noise in measuring.
   *  Since the value of the gravity doesn't matter (only ratio matters), we can
   *  assign the noise bias to the gravity noise bias. So acc_z_off is always zero.
   **/
  switch (gravityDir) {
    case X:
      acc_x_off = 0;
      break;
    case Y:
      acc_y_off = 0;
      break;
    case Z:
      acc_z_off = 0;
      break;
    case X_NEG:
      acc_x_off = 0;
      break;
    case Y_NEG:
      acc_y_off = 0;
      break;
    case Z_NEG:
      acc_z_off = 0;
      break;

  }

  gyro_x_off = gyro_x / (float)_num;
  gyro_y_off = gyro_y / (float)_num;
  gyro_z_off = gyro_z / (float)_num;
}

float Pose::GetGravity(void) {
  float acce = 0;
  const int32_t NUM = 100;
  for (int i = 0; i < NUM; i++) {
    switch (gravityDir) {
      case X:
        acce += imu->acce.x;
        break;
      case Y:
        acce += imu->acce.y;
        break;
      case Z:
        acce += imu->acce.z;
        break;
      case X_NEG:
        acce += imu->acce.x;
        break;
      case Y_NEG:
        acce += imu->acce.y;
        break;
      case Z_NEG:
        acce += imu->acce.z;
        break;
    }
    osDelay(10);
  }
  return acce / (float)NUM;
}

void Pose::SetGravityDir(uint8_t _gd) {
  if (_gd == X || _gd == Y || _gd == Z || 
      _gd == X_NEG || _gd == Y_NEG || _gd == Z_NEG) {
    gravityDir = _gd;
  }
}

void Pose::SetOffset(float _acc_x_off, float _acc_y_off, float _acc_z_off, float _gyro_x_off,
                     float _gyro_y_off, float _gyro_z_off) {
  acc_x_off = _acc_x_off;
  acc_y_off = _acc_y_off;
  acc_z_off = _acc_z_off;
  gyro_x_off = _gyro_x_off;
  gyro_y_off = _gyro_y_off;
  gyro_z_off = _gyro_z_off;
}

float Pose::GetPitch(void) { return pitch; }

float Pose::GetRoll(void) { return roll; }

float Pose::GetYaw(void) { return yaw; }

void Pose::SetAlpha(float _alpha) {
  if (_alpha > 1.0 || _alpha < 0.0) {
    RM_EXPECT_TRUE(false, "Invalid complementary filter weight");
  }
  alpha = _alpha;
}

void Pose::ComplementaryFilterUpdate(void) {
  if (gravityDir == X) {
    // compute pitch and roll based on acce meter
    pitchAcc = atan2f(imu->acce.z - acc_z_off, imu->acce.x - acc_x_off);
    rollAcc = atan2f(imu->acce.y - acc_y_off, imu->acce.x - acc_x_off);

    // estimate pose from gyro and acce
    pitch = alpha * (pitch + (imu->gyro.y - gyro_y_off) * (float)(imu->timestamp - timestamp) /
                                 USEC_TO_SEC) +
            (1.0 - alpha) * pitchAcc;

    roll = alpha * (roll + (imu->gyro.z - gyro_z_off) * (float)(imu->timestamp - timestamp) /
                               USEC_TO_SEC) -
           (1.0 - alpha) * rollAcc;

    // yaw cannot rely on acce.
    yaw = yaw + (imu->gyro.x - gyro_x_off) * (float)(imu->timestamp - timestamp) / USEC_TO_SEC;
    // limit yaw to -Pi to Pi
    yaw = wrap<float>(yaw, -PI, PI);

  } else if (gravityDir == Y) {
    // compute pitch and roll based on acce meter
    pitchAcc = atan2f(imu->acce.x - acc_x_off, imu->acce.y - acc_y_off);
    rollAcc = atan2f(imu->acce.z - acc_z_off, imu->acce.y - acc_y_off);

    // estimate pose from gyro and acce
    pitch = alpha * (pitch + (imu->gyro.z - gyro_z_off) * (float)(imu->timestamp - timestamp) /
                                 USEC_TO_SEC) +
            (1.0 - alpha) * pitchAcc;

    roll = alpha * (roll + (imu->gyro.x - gyro_x_off) * (float)(imu->timestamp - timestamp) /
                               USEC_TO_SEC) -
           (1.0 - alpha) * rollAcc;

    // yaw cannot rely on acce.
    yaw = yaw + (imu->gyro.y - gyro_y_off) * (float)(imu->timestamp - timestamp) / USEC_TO_SEC;
    // limit yaw to -Pi to Pi
    yaw = wrap<float>(yaw, -PI, PI);

  } else if (gravityDir == Z) {
    // compute pitch and roll based on acce meter
    pitchAcc = atan2f(imu->acce.y - acc_y_off, imu->acce.z - acc_z_off);
    rollAcc = atan2f(imu->acce.x - acc_x_off, imu->acce.z - acc_z_off);

    // estimate pose from gyro and acce
    pitch = alpha * (pitch + (imu->gyro.x - gyro_x_off) * (float)(imu->timestamp - timestamp) /
                                 USEC_TO_SEC) +
            (1.0 - alpha) * pitchAcc;

    roll = alpha * (roll + (imu->gyro.y - gyro_y_off) * (float)(imu->timestamp - timestamp) /
                               USEC_TO_SEC) -
           (1.0 - alpha) * rollAcc;

    // yaw cannot rely on acce.
    yaw = yaw + (imu->gyro.z - gyro_z_off) * (float)(imu->timestamp - timestamp) / USEC_TO_SEC;
    // limit yaw to -Pi to Pi
    yaw = wrap<float>(yaw, -PI, PI);
  }
  if (gravityDir == X_NEG) {
    // compute pitch and roll based on acce meter
    pitchAcc = atan2f(imu->acce.z + acc_z_off, imu->acce.x + acc_x_off);
    rollAcc = atan2f(imu->acce.y + acc_y_off, imu->acce.x + acc_x_off);

    // estimate pose from gyro and acce
    pitch = alpha * (pitch + (imu->gyro.y + gyro_y_off) * (float)(imu->timestamp - timestamp) /
                                 USEC_TO_SEC) +
            (1.0 - alpha) * pitchAcc;

    roll = alpha * (roll + (imu->gyro.z + gyro_z_off) * (float)(imu->timestamp - timestamp) /
                               USEC_TO_SEC) -
           (1.0 - alpha) * rollAcc;

    // yaw cannot rely on acce.
    yaw = yaw + (imu->gyro.x - gyro_x_off) * (float)(imu->timestamp - timestamp) / USEC_TO_SEC;
    // limit yaw to -Pi to Pi
    yaw = wrap<float>(yaw, -PI, PI);

  } else if (gravityDir == Y_NEG) {
    // compute pitch and roll based on acce meter
    pitchAcc = atan2f(imu->acce.x + acc_x_off, imu->acce.y + acc_y_off);
    rollAcc = atan2f(imu->acce.z + acc_z_off, imu->acce.y + acc_y_off);

    // estimate pose from gyro and acce
    pitch = alpha * (pitch + (imu->gyro.z + gyro_z_off) * (float)(imu->timestamp - timestamp) /
                                 USEC_TO_SEC) +
            (1.0 - alpha) * pitchAcc;

    roll = alpha * (roll + (imu->gyro.x + gyro_x_off) * (float)(imu->timestamp - timestamp) /
                               USEC_TO_SEC) -
           (1.0 - alpha) * rollAcc;

    // yaw cannot rely on acce.
    yaw = yaw + (imu->gyro.y - gyro_y_off) * (float)(imu->timestamp - timestamp) / USEC_TO_SEC;
    // limit yaw to -Pi to Pi
    yaw = wrap<float>(yaw, -PI, PI);

  } else if (gravityDir == Z_NEG) {
    // compute pitch and roll based on acce meter
    pitchAcc = atan2f(imu->acce.y + acc_y_off, imu->acce.z + acc_z_off);
    rollAcc = atan2f(imu->acce.x + acc_x_off, imu->acce.z + acc_z_off);

    // estimate pose from gyro and acce
    pitch = alpha * (pitch + (imu->gyro.x + gyro_x_off) * (float)(imu->timestamp - timestamp) /
                                 USEC_TO_SEC) +
            (1.0 - alpha) * pitchAcc;

    roll = alpha * (roll + (imu->gyro.y + gyro_y_off) * (float)(imu->timestamp - timestamp) /
                               USEC_TO_SEC) -
           (1.0 - alpha) * rollAcc;

    // yaw cannot rely on acce.
    yaw = yaw + (imu->gyro.z - gyro_z_off) * (float)(imu->timestamp - timestamp) / USEC_TO_SEC;
    // limit yaw to -Pi to Pi
    yaw = wrap<float>(yaw, -PI, PI);
  }

  // update timestamp
  timestamp = imu->timestamp;
}

}  // namespace control
