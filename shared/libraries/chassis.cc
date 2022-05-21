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

namespace control {

Chassis::Chassis(const chassis_t chassis) : pids_() {
  // acquired from user
  model_ = chassis.model;

  // data initialization using acquired model
  switch (chassis.model) {
    case CHASSIS_STANDARD_ZERO:
    case CHASSIS_STANDARD_2022_ALPHA:
      motors_ = new MotorCANBase*[FourWheel::motor_num];
      motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
      motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
      motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
      motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

      {
        float* pid_param = new float[3]{20, 4, 3};  // {5, 3, 0.1}
        pids_[FourWheel::front_left].Reinit(pid_param);
        pids_[FourWheel::front_right].Reinit(pid_param);
        pids_[FourWheel::back_left].Reinit(pid_param);
        pids_[FourWheel::back_right].Reinit(pid_param);
      }

      power_limit_ = new PowerLimit(FourWheel::motor_num, 40, 0.0000000000000000002, 0);

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
    case CHASSIS_STANDARD_2022_ALPHA:
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
    case CHASSIS_STANDARD_2022_ALPHA:
      constexpr int MAX_ABS_CURRENT = 12288;  // refer to MotorM3508 for details
      float move_sum = fabs(x_speed) + fabs(y_speed) + fabs(turn_speed);
      float scale = move_sum >= MAX_ABS_CURRENT ? MAX_ABS_CURRENT / move_sum : 1;

      speeds_[FourWheel::front_left] = scale * (y_speed + x_speed + turn_speed);
      speeds_[FourWheel::back_left] = scale * (y_speed - x_speed + turn_speed);
      speeds_[FourWheel::front_right] = -scale * (y_speed - x_speed - turn_speed);
      speeds_[FourWheel::back_right] = -scale * (y_speed + x_speed - turn_speed);
      break;
  }
}

void Chassis::Update() {
  switch (model_) {
    case CHASSIS_STANDARD_ZERO:
    case CHASSIS_STANDARD_2022_ALPHA:
      float vel_real[FourWheel::motor_num];
      float PID_output[FourWheel::motor_num];
      float output[FourWheel::motor_num];

      vel_real[FourWheel::front_left] = motors_[FourWheel::front_left]->GetOmega();
      vel_real[FourWheel::back_left] = motors_[FourWheel::back_left]->GetOmega();
      vel_real[FourWheel::front_right] = motors_[FourWheel::front_right]->GetOmega();
      vel_real[FourWheel::back_right] = motors_[FourWheel::back_right]->GetOmega();

      PID_output[FourWheel::front_left] = pids_[FourWheel::front_left].ComputeOutput(motors_[FourWheel::front_left]->GetOmegaDelta(speeds_[FourWheel::front_left]));
      PID_output[FourWheel::back_left] = pids_[FourWheel::back_left].ComputeOutput(motors_[FourWheel::back_left]->GetOmegaDelta(speeds_[FourWheel::back_left]));
      PID_output[FourWheel::front_right] = pids_[FourWheel::front_right].ComputeOutput(motors_[FourWheel::front_right]->GetOmegaDelta(speeds_[FourWheel::front_right]));
      PID_output[FourWheel::back_right] = pids_[FourWheel::back_right].ComputeOutput(motors_[FourWheel::back_right]->GetOmegaDelta(speeds_[FourWheel::back_right]));

//      print("After:  %.3f %.3f %.3f %.3f\r\n", output[FourWheel::front_left], output[FourWheel::back_left], output[FourWheel::front_right], output[FourWheel::back_right]);
//      print("Before: %.3f %.3f %.3f %.3f\r\n", PID_output[FourWheel::front_left], PID_output[FourWheel::back_left], PID_output[FourWheel::front_right], PID_output[FourWheel::back_right]);
      power_limit_->Output(vel_real, PID_output, output);
//      print("After:  %.3f %.3f %.3f %.3f\r\n", output[FourWheel::front_left], output[FourWheel::back_left], output[FourWheel::front_right], output[FourWheel::back_right]);


      motors_[FourWheel::front_left]->SetOutput(control::ClipMotorRange(output[FourWheel::front_left]));
      motors_[FourWheel::back_left]->SetOutput(control::ClipMotorRange(output[FourWheel::back_left]));
      motors_[FourWheel::front_right]->SetOutput(control::ClipMotorRange(output[FourWheel::front_right]));
      motors_[FourWheel::back_right]->SetOutput(control::ClipMotorRange(output[FourWheel::back_right]));

//      motors_[FourWheel::front_left]->SetOutput(
//          pids_[FourWheel::front_left].ComputeConstraintedOutput(
//              motors_[FourWheel::front_left]->GetOmegaDelta(speeds_[FourWheel::front_left])));
//      motors_[FourWheel::back_left]->SetOutput(
//          pids_[FourWheel::back_left].ComputeConstraintedOutput(
//              motors_[FourWheel::back_left]->GetOmegaDelta(speeds_[FourWheel::back_left])));
//      motors_[FourWheel::front_right]->SetOutput(
//          pids_[FourWheel::front_right].ComputeConstraintedOutput(
//              motors_[FourWheel::front_right]->GetOmegaDelta(speeds_[FourWheel::front_right])));
//      motors_[FourWheel::back_right]->SetOutput(
//          pids_[FourWheel::back_right].ComputeConstraintedOutput(
//              motors_[FourWheel::back_right]->GetOmegaDelta(speeds_[FourWheel::back_right])));
      break;
  }
}

} /* namespace control */
