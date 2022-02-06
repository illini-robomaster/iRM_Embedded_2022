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
  constexpr int MIN = -32768; /* Minimum that a 16-bit number can represent */
  constexpr int MAX = 32767;  /* Maximum that a 16-bit number can represent */
  return clip<int>((int)arm_pid_f32(&pid_f32_, error), MIN, MAX);
}

void PIDController::Reinit(float kp, float ki, float kd) {
  pid_f32_.Kp = kp;
  pid_f32_.Ki = ki;
  pid_f32_.Kd = kd;
  arm_pid_init_f32(&pid_f32_, 0);
}

void PIDController::Reinit(float* param) { Reinit(param[0], param[1], param[2]); }

void PIDController::Reset() { arm_pid_init_f32(&pid_f32_, 1); }

} /* namespace control */
