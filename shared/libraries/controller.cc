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

#include "controller.h"

#include "utils.h"

namespace control {

typedef struct {
  float kp;
  float ki;
  float kd;
} pid_t;

PIDController::PIDController() {
  pid_f32_.Kp = 0;
  pid_f32_.Ki = 0;
  pid_f32_.Kd = 0;
  arm_pid_init_f32(&pid_f32_, 1);
}

PIDController::PIDController(float kp, float ki, float kd) {
  pid_f32_.Kp = kp;
  pid_f32_.Ki = ki;
  pid_f32_.Kd = kd;
  arm_pid_init_f32(&pid_f32_, 1);
}

PIDController::PIDController(float* param) : PIDController(param[0], param[1], param[2]) {}

float PIDController::ComputeOutput(float error) { return arm_pid_f32(&pid_f32_, error); }

int16_t PIDController::ComputeConstraintedOutput(float error) {
  /*
   * CAN protocal uses a 16-bit signed number to drive the motors, so this version
   * of the output computation can make sure that no unexpected behavior (overflow)
   * can happen.
   */
  constexpr int MOTOR_MIN = -32768; /* Minimum that a 16-bit number can represent */
  constexpr int MOTOR_MAX = 32767;  /* Maximum that a 16-bit number can represent */
  return clip<int>((int)arm_pid_f32(&pid_f32_, error), MOTOR_MIN, MOTOR_MAX);
}

void PIDController::Reinit(float kp, float ki, float kd) {
  pid_f32_.Kp = kp;
  pid_f32_.Ki = ki;
  pid_f32_.Kd = kd;
  arm_pid_init_f32(&pid_f32_, 0);
}

void PIDController::Reinit(float* param) { Reinit(param[0], param[1], param[2]); }

void PIDController::Reset() { arm_pid_reset_f32(&pid_f32_); }

OutputConstraintedPIDController::OutputConstraintedPIDController() {
  Reinit(0, 0, 0);
  Reset();
}

OutputConstraintedPIDController::OutputConstraintedPIDController(float kp, float ki, float kd) {
  Reinit(kp, ki, kd);
  Reset();
}

OutputConstraintedPIDController::OutputConstraintedPIDController(float* param) {
  Reinit(param[0], param[1], param[2]);
  Reset();
}

float OutputConstraintedPIDController::ComputeOutput(float error, int max_out) {
  constexpr int MOTOR_MAX = 32767;
  max_out = clip<float>(max_out, 0, MOTOR_MAX);
  float bias = ki_ * cumulated_err_ - kd_ * last_err_;
  float error_max = (max_out - bias) / param_sum_;
  float error_min = (-max_out - bias) / param_sum_;
  error = clip<float>(error, error_min, error_max);

  cumulated_err_ += error;
  float out = kp_ * error + ki_ * cumulated_err_ + kd_ * (error - last_err_);
  last_err_ = error;
  return out;
}

void OutputConstraintedPIDController::Reinit(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  param_sum_ = kp_ + ki_ + kd_;
}

void OutputConstraintedPIDController::Reinit(float* param) { Reinit(param[0], param[1], param[2]); }

void OutputConstraintedPIDController::Reset() {
  cumulated_err_ = 0;
  last_err_ = 0;
}

} /* namespace control */
