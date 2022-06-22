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

#include "controller.h"
#include "motor.h"
#include "power_limit.h"

constexpr uint16_t MOTOR_NUM = 4;

namespace control {

typedef struct {
  control::SteeringMotor* fl_steer_motor = nullptr;
  control::SteeringMotor* fr_steer_motor = nullptr;
  control::SteeringMotor* bl_steer_motor = nullptr;
  control::SteeringMotor* br_steer_motor = nullptr;

  control::MotorCANBase* fl_wheel_motor = nullptr;
  control::MotorCANBase* fr_wheel_motor = nullptr;
  control::MotorCANBase* bl_wheel_motor = nullptr;
  control::MotorCANBase* br_wheel_motor = nullptr;

  double radius = 1.0;

  double fl_calibration_offset;
  double fr_calibration_offset;
  double bl_calibration_offset;
  double br_calibration_offset;
} steering_chassis_t;

class SteeringChassis {
 public:
  SteeringChassis(const steering_chassis_t* chassis);

  ~SteeringChassis();

  // right -> positive, left -> negative
  void SetXSpeed(double _vx);

  // front -> positive, back -> negative
  void SetYSpeed(double _vy);

  // counterclockwise -> positive
  void SetWSpeed(double _vw);

  void Update(float power_limit, float chassis_power, float chassis_power_buffer);

 private:
  // current velocity
  // right -> positive, left -> negative
  // front -> positive, back -> negative
  // counterclockwise -> positive
  double vx;
  double vy;
  double vw;

  // radius of the vehicle from the center to the wheels
  double radius;

  // current steering pos of the 4 wheels
  double theta0;
  double theta1;
  double theta2;
  double theta3;

  // same as class Chassis
  ConstrainedPID pids[4];
  PowerLimit* power_limit;
  power_limit_t power_limit_info;
  const steering_chassis_t* chassis;

};  // class SteeringChassis ends

}  // namespace control
