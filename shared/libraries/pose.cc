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

// EKF model generated with symforce
#include "gen/ahrs_process.h"
#include "gen/ahrs_update_accel.h"
#include "gen/ahrs_update_mag.h"
#include "gen/ahrs_update_retract.h"

// Factor from us to s
static const float USEC_TO_SEC = 1000000.0;

namespace control {

Pose::Pose(bsp::MPU6500* _imu) : imu(_imu) {
  if (_imu == nullptr) {
    RM_ASSERT_TRUE(false, "invalid imu");
  }
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
}

void Pose::Calibrate(void) {
  // measure 100 times to compute the average
  Calibrate(100);
}

void Pose::Calibrate(int16_t _num) {
  float acce_x = 0;
  float acce_y = 0;
  float gyro_x = 0;
  float gyro_y = 0;
  float gyro_z = 0;

  // average NUM times acc and gyro reads
  for (int i = 0; i < _num; i++) {
    acce_x += imu->acce.x();
    acce_y += imu->acce.y();
    gyro_x += imu->gyro.x();
    gyro_y += imu->gyro.y();
    gyro_z += imu->gyro.z();
    osDelay(10);
  }

  acc_x_off = acce_x / (float)_num;
  acc_y_off = acce_y / (float)_num;

  /** Explanation: For calibration, we want the mean of noise distribution (bias)
   *  to be zeroed when IMU is placed on a flat plane.
   *  Note that acc_z_off should have a constant gravity offset on a flat plane,
   *  which also has a noise in measuring.
   *  Since the value of the gravity doesn't matter (only ratio matters), we can
   *  assign the noise bias to the gravity noise bias. So acc_z_off is always zero.
   **/
  acc_z_off = 0;

  gyro_x_off = gyro_x / (float)_num;
  gyro_y_off = gyro_y / (float)_num;
  gyro_z_off = gyro_z / (float)_num;
}

float Pose::GetGravity(void) {
  float acce_z = 0;
  const int32_t NUM = 100;
  for (int i = 0; i < NUM; i++) {
    acce_z += imu->acce.z();
    osDelay(10);
  }
  return acce_z / (float)NUM;
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
  // compute pitch and roll based on acce meter
  pitchAcc = atan2f(imu->acce.y() - acc_y_off, imu->acce.z() - acc_z_off);
  rollAcc = atan2f(imu->acce.x() - acc_x_off, imu->acce.z() - acc_z_off);

  // estimate pose from gyro and acce
  pitch = alpha * (pitch + (imu->gyro.x() - gyro_x_off) * (float)(imu->timestamp - timestamp) /
                               USEC_TO_SEC) +
          (1.0 - alpha) * pitchAcc;

  roll = alpha * (roll + (imu->gyro.y() - gyro_y_off) * (float)(imu->timestamp - timestamp) /
                             USEC_TO_SEC) -
         (1.0 - alpha) * rollAcc;

  // yaw cannot rely on acce.
  // TODO fuse yaw with
  yaw = yaw + (imu->gyro.z() - gyro_z_off) * (float)(imu->timestamp - timestamp) / USEC_TO_SEC;
  // limit yaw to -Pi to Pi
  yaw = wrap<float>(yaw, -PI, PI);

  // update timestamp
  timestamp = imu->timestamp;
}

AHRSFilter::AHRSFilter(const Eigen::Vector3f &gyro_noise,
                       const Eigen::Vector3f &accel_noise,
                       const Eigen::Vector3f &mag_noise,
                       const Eigen::Vector3f &gyro_bias_noise,
                       const Eigen::Vector3f &accel_bias_noise,
                       const Eigen::Vector3f &mag_bias_noise,
                       const Eigen::Vector3f &gravity_field,
                       const Eigen::Vector3f &magnetic_field,
                       const osMutexId_t &lock)
    : gyro_noise_(gyro_noise),
      accel_noise_(accel_noise),
      mag_noise_(mag_noise),
      gyro_bias_noise_(gyro_bias_noise),
      accel_bias_noise_(accel_bias_noise),
      mag_bias_noise_(mag_bias_noise),
      g_(gravity_field),
      m_(magnetic_field),
      lock_(lock) {}

void AHRSFilter::Initialize(const Eigen::Quaternionf &pose,
                            const Eigen::Vector3f &accel_bias,
                            const Eigen::Vector3f &gyro_bias,
                            const Eigen::Vector3f &mag_bias,
                            const Eigen::Vector3f &pose_prior,
                            const Eigen::Vector3f &accel_bias_prior,
                            const Eigen::Vector3f &gyro_bias_prior,
                            const Eigen::Vector3f &mag_bias_prior,
                            const Eigen::Vector3f &gyro_body,
                            const uint32_t timestamp) {
  // set states
  states_.segment<4>(0) = pose.coeffs();
  states_.segment<3>(4) = accel_bias;
  states_.segment<3>(7) = gyro_bias;
  states_.segment<3>(10) = mag_bias;

  // set covariance
  states_cov_.diagonal().segment<3>(0) = pose_prior;
  states_cov_.diagonal().segment<3>(3) = accel_bias_prior;
  states_cov_.diagonal().segment<3>(6) = gyro_bias_prior;
  states_cov_.diagonal().segment<3>(9) = mag_bias_prior;

  // set one gyro measurement
  latest_gyro_ = gyro_body;

  // set timestamp
  latest_time_ = timestamp;
}

void AHRSFilter::Predict(const uint32_t timestamp,
                         Eigen::Matrix<float, 13, 1>* const proc_states,
                         Eigen::Matrix<float, 12, 12>* const proc_states_cov) const {
  // update only when dt > 0
  if (timestamp > latest_time_) {
    const float dt = (timestamp - latest_time_) * 1e-6;
    sym::AhrsProcess<float>(states_, states_cov_, latest_gyro_, dt,
                            gyro_noise_, accel_bias_noise_, gyro_bias_noise_, mag_bias_noise_,
                            proc_states, proc_states_cov);
  } else {
    *proc_states = states_;
    *proc_states_cov = states_cov_;
  }
}

void AHRSFilter::MeasureGyro(const Eigen::Vector3f &gyro_body, const uint32_t timestamp) {
  Eigen::Matrix<float, 13, 1> next_states;
  Eigen::Matrix<float, 12, 12> next_states_cov;
  Predict(timestamp, &next_states, &next_states_cov);

  osMutexAcquire(lock_, osWaitForever);
  states_ = next_states;
  states_cov_ = .5 * (next_states_cov.transpose() + next_states_cov);
  latest_gyro_ = gyro_body;
  latest_time_ = timestamp;
  osMutexRelease(lock_);
}

void AHRSFilter::MeasureAccel(const Eigen::Vector3f &accel_body, const uint32_t timestamp) {
  Eigen::Matrix<float, 13, 1> x;
  Eigen::Matrix<float, 12, 12> P;
  Predict(timestamp, &x, &P);

  Eigen::Vector3f y;
  Eigen::Matrix3f S;
  Eigen::Matrix<float, 3, 12> H;
  sym::AhrsUpdateAccel<float>(x, P,
                              accel_body, accel_noise_, g_,
                              &y, &S, &H);

  // compute kalman update
  const Eigen::Matrix<float, 12, 3> K = S.llt().solve(H * P).transpose();
  const Eigen::Matrix<float, 12, 1> delta = K * y;

  osMutexAcquire(lock_, osWaitForever);
  latest_time_ = timestamp;
  sym::AhrsUpdateRetract<float>(x, delta, &states_);
  states_cov_ = P - K * H * P;
  states_cov_ = .5 * (states_cov_.transpose() + states_cov_).eval();
  osMutexRelease(lock_);
}

void AHRSFilter::MeasureMag(const Eigen::Vector3f &mag_body, const uint32_t timestamp) {
  Eigen::Matrix<float, 13, 1> x;
  Eigen::Matrix<float, 12, 12> P;
  Predict(timestamp, &x, &P);

  Eigen::Vector3f y;
  Eigen::Matrix3f S;
  Eigen::Matrix<float, 3, 12> H;
  sym::AhrsUpdateMag<float>(x, P,
                            mag_body, mag_noise_, m_,
                            &y, &S, &H);

  // compute kalman update
  const Eigen::Matrix<float, 12, 3> K = S.llt().solve(H * P).transpose();
  const Eigen::Matrix<float, 12, 1> delta = K * y;

  osMutexAcquire(lock_, osWaitForever);
  latest_time_ = timestamp;
  sym::AhrsUpdateRetract<float>(x, delta, &states_);
  states_cov_ = P - K * H * P;
  states_cov_ = .5 * (states_cov_.transpose() + states_cov_).eval();
  osMutexRelease(lock_);
}

void AHRSFilter::GetLatestState(Eigen::Quaternionf* const pose,
                                Eigen::Vector3f* const accel_bias,
                                Eigen::Vector3f* const gyro_bias,
                                Eigen::Vector3f* const mag_bias) const {
  osMutexAcquire(lock_, osWaitForever);
  if (pose) pose->coeffs() = states_.segment<4>(0);
  if (accel_bias) *accel_bias = states_.segment<3>(4);
  if (gyro_bias) *gyro_bias = states_.segment<3>(7);
  if (mag_bias) *mag_bias = states_.segment<3>(10);
  osMutexRelease(lock_);
}

uint32_t AHRSFilter::GetLatestTime() const { return latest_time_; }

}  // namespace control
