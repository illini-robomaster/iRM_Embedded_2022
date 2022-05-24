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

#include "fortress.h"

namespace control {

Fortress::Fortress(MotorCANBase** motors, MotorCANBase* yaw_motor) {
  motors_ = new MotorCANBase*[FORTRESS_MOTOR_NUM];
  float* pid_param = new float[3]{20, 1, 2};
  for (int i = 0; i < FORTRESS_MOTOR_NUM; ++i) {
    motors_[i] = motors[i];
    pids_[i].Reinit(pid_param);
  }

  yaw_motor_ = yaw_motor;
  float* pid_yaw_param = new float[3]{20, 0, 0};
  yaw_pid_.Reinit(pid_yaw_param);
}

Fortress::~Fortress() {
  for (int i = 0; i < FORTRESS_MOTOR_NUM; ++i)
    motors_[i] = nullptr;
  delete[] motors_;
  motors_ = nullptr;
}

void Fortress::Up() {
  for (int i = 0; i < FORTRESS_MOTOR_NUM; ++i) {
    motors_[i]->SetOutput(pids_[i].ComputeConstrainedOutput(motors_[i]->GetOmegaDelta(FORTRESS_RISE_SPEED)));
  }
}

void Fortress::Down() {
  for (int i = 0; i < FORTRESS_MOTOR_NUM; ++i) {
    motors_[i]->SetOutput(pids_[i].ComputeConstrainedOutput(motors_[i]->GetOmegaDelta(-FORTRESS_RISE_SPEED)));
  }
}

void Fortress::StopRise() {
  for (int i = 0; i < FORTRESS_MOTOR_NUM; ++i) {
    motors_[i]->SetOutput(pids_[i].ComputeConstrainedOutput(motors_[i]->GetOmegaDelta(0)));
  }
}

void Fortress::Rotate() {
  yaw_motor_->SetOutput((yaw_pid_.ComputeConstrainedOutput(yaw_motor_->GetOmegaDelta(FORTRESS_ROTATE_SPEED))));
}

void Fortress::StopRotate() {
  yaw_motor_->SetOutput((yaw_pid_.ComputeConstrainedOutput(yaw_motor_->GetOmegaDelta(0))));
}

}
