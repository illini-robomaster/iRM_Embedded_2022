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

#include "chassis.h"

#include "bsp_error_handler.h"
#include "can.h"
#include "cmath"

namespace control {

Chassis::Chassis(const chassis_t chassis) : pids_() {
  // acquired from user
  model_ = chassis.model;

  // data initialization using acquired model
  switch (chassis.model) {
    case CHASSIS_STANDARD_ZERO:
      motors_ = new MotorCANBase*[FourWheel::motor_num];
      motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
      motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
      motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
      motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

      {
        float* pid_param = new float[3]{20, 8, 2};  // {5, 3, 0.1}
        pids_[FourWheel::front_left].Reinit(pid_param);
        pids_[FourWheel::front_right].Reinit(pid_param);
        pids_[FourWheel::back_left].Reinit(pid_param);
        pids_[FourWheel::back_right].Reinit(pid_param);
      }

      speeds_ = new float[FourWheel::motor_num];
      for (int i = 0; i < FourWheel::motor_num; i++) speeds_[i] = 0;
      break;
    default:
      RM_ASSERT_TRUE(false, "No chassis type specified");
  }
}

Chassis::~Chassis() {
  switch (model_) {
    case CHASSIS_STANDARD_ZERO:
      motors_[FourWheel::front_left] = nullptr;
      motors_[FourWheel::front_right] = nullptr;
      motors_[FourWheel::back_left] = nullptr;
      motors_[FourWheel::back_right] = nullptr;
      delete[] motors_;
      motors_ = nullptr;

      delete[] speeds_;
      speeds_ = nullptr;
      break;
  }
}

void Chassis::SetSpeed(const float x_speed, const float y_speed, const float turn_speed) {
  switch (model_) {
    case CHASSIS_STANDARD_ZERO:
      constexpr int MAX_ABS_CURRENT = 12288;  // refer to MotorM3508 for details
      float move_sum = fabs(x_speed) + fabs(y_speed) + fabs(turn_speed);
      float scale = move_sum >= MAX_ABS_CURRENT ? MAX_ABS_CURRENT / move_sum : 1;

      speeds_[FourWheel::front_left] = scale * (-y_speed + x_speed + turn_speed);
      speeds_[FourWheel::back_left] = scale * (y_speed - x_speed + turn_speed);
      speeds_[FourWheel::front_right] = -scale * (-y_speed - x_speed - turn_speed);
      speeds_[FourWheel::back_right] = -scale * (y_speed + x_speed - turn_speed);
      break;
  }
}

void Chassis::Update() {
  switch (model_) {
    case CHASSIS_STANDARD_ZERO:
      constexpr float CURRENT_MAX = 100 / 6 * 7 / 24 * 1000;
      float outputs[FourWheel::motor_num];
      float requested_current = 0;
      for (int i = 0; i < FourWheel::motor_num; i++) {
        outputs[i] = pids_[i].TryComputeOutput(motors_[i]->GetOmegaDelta(speeds_[i]));
        outputs[i] = clip<float>(outputs[i], -12288, 12288);
        requested_current += abs(outputs[i]);
      }
      float k = requested_current > CURRENT_MAX ? CURRENT_MAX / requested_current : 1;
      print("% 7.5f % 7.5f \r\n", requested_current, k);
      for (int i = 0; i < FourWheel::motor_num; i++) {
        motors_[i]->SetOutput(
            pids_[i].ComputeOutput(motors_[i]->GetOmegaDelta(speeds_[i]), k * abs(outputs[i])));
      }
      break;
  }
}

} /* namespace control */
