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

#include "steering.h"

#include <cmath>

#include "controller.h"
#include "motor.h"

namespace control {
SteeringChassis::SteeringChassis(const steering_chassis_t* _chassis) : chassis(_chassis) {
  radius = _chassis->radius;
  vx = 0.0;
  vy = 0.0;
  vw = 0.0;

  float* PID_PARAMS = new float[3]{40, 3, 0};
  float MOTOR_MAX_IOUT = 2000;
  float MOTOR_MAX_OUT = 20000;
  for (int i = 0; i < MOTOR_NUM; i++) {
    pids[i].Reinit(PID_PARAMS, MOTOR_MAX_IOUT, MOTOR_MAX_OUT);
  }
}

SteeringChassis::~SteeringChassis() { chassis = nullptr; }

void SteeringChassis::SetXSpeed(double _vx) { vx = _vx; }

void SteeringChassis::SetYSpeed(double _vy) { vy = _vy; }

void SteeringChassis::SetWSpeed(double _vw) { vw = _vw; }

void SteeringChassis::Update(float _power_limit, float _chassis_power,
                             float _chassis_power_buffer) {
  // Update Steer
  // compute angle and speed
  double v0 = sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
  double v1 = sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
  double v2 = sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
  double v3 = sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));

  double _theta0 = atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4));
  double _theta1 = atan2(vy - vw * cos(PI / 4), vx + vw * sin(PI / 4));
  double _theta2 = atan2(vy + vw * cos(PI / 4), vx + vw * sin(PI / 4));
  double _theta3 = atan2(vy + vw * cos(PI / 4), vx - vw * sin(PI / 4));

  // update with relative angles (-180 - 180)
  chassis->fl_steer_motor->TurnRelative(clip<double>(_theta0 - theta0, -PI, PI));
  chassis->fr_steer_motor->TurnRelative(clip<double>(_theta1 - theta1, -PI, PI));
  chassis->bl_steer_motor->TurnRelative(clip<double>(_theta2 - theta2, -PI, PI));
  chassis->br_steer_motor->TurnRelative(clip<double>(_theta3 - theta3, -PI, PI));

  theta0 = _theta0;
  theta1 = _theta1;
  theta2 = _theta2;
  theta3 = _theta3;

  // Update Wheels
  float PID_output[MOTOR_NUM];

  // compute PID output
  PID_output[0] = pids[0].ComputeOutput(chassis->fl_wheel_motor->GetOmegaDelta(v0));
  PID_output[1] = pids[1].ComputeOutput(chassis->fr_wheel_motor->GetOmegaDelta(v1));
  PID_output[2] = pids[2].ComputeOutput(chassis->bl_wheel_motor->GetOmegaDelta(v2));
  PID_output[3] = pids[3].ComputeOutput(chassis->br_wheel_motor->GetOmegaDelta(v3));

  float output[MOTOR_NUM];
  // compute power limit
  power_limit_info.power_limit = _power_limit;
  power_limit_info.WARNING_power = _power_limit * 0.9;
  power_limit_info.WARNING_power_buff = 50;
  power_limit_info.buffer_total_current_limit = 3500 * MOTOR_NUM;
  power_limit_info.power_total_current_limit = 5000 * MOTOR_NUM / 80.0 * _power_limit;
  power_limit->Output(power_limit_info, _chassis_power, _chassis_power_buffer, PID_output, output);

  // set final output
  chassis->fl_wheel_motor->SetOutput(control::ClipMotorRange(output[0]));
  chassis->fr_wheel_motor->SetOutput(control::ClipMotorRange(output[1]));
  chassis->bl_wheel_motor->SetOutput(control::ClipMotorRange(output[2]));
  chassis->br_wheel_motor->SetOutput(control::ClipMotorRange(output[3]));
}

}  // namespace control
