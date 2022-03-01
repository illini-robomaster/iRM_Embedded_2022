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

#include "bsp_print.h"
#include "motor.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"

BoolEdgeDetector shoot_detector(false);
control::MotorPWMBase* left_fly_motor = nullptr;
control::MotorPWMBase* right_fly_motor = nullptr;

bsp::CAN* can1 = nullptr;
control::MotorCANBase* load_motor = nullptr;
control::MotorCANBase* pitch_motor_1 = nullptr;
control::MotorCANBase* pitch_motor_2 = nullptr;
control::MotorCANBase* yaw_motor = nullptr;

remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  // See pwm example
  left_fly_motor = new control::MotorPWMBase(&htim1, 1, 1000000, 50, 1000);
  right_fly_motor = new control::MotorPWMBase(&htim1, 4, 1000000, 50, 1000);
  can1 = new bsp::CAN(&hcan1, 0x201);

  load_motor = new control::Motor3508(can1, 0x201);
  pitch_motor_1 = new control::Motor3508(can1, 0x202);
  pitch_motor_2 = new control::Motor3508(can1, 0x203);
  yaw_motor = new control::Motor3508(can1, 0x204);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);

  control::MotorCANBase* motors[] = {load_motor, pitch_motor_1, pitch_motor_2, yaw_motor};
  control::PIDController pid_load(20, 15, 30);
  control::PIDController pid_yaw(20, 15, 30);
  control::PIDController pid_pitch_1(20, 15, 30);
  control::PIDController pid_pitch_2(20, 15, 30);

  float max_speed = 300;
  float speed_load;
  float speed_yaw;
  float speed_pitch;

  float diff_load;
  int16_t out_load;

  float diff_yaw;
  int16_t out_yaw;

  float diff_pitch_1;
  int16_t out_pitch_1;

  float diff_pitch_2;
  int16_t out_pitch_2;

  while (true) {
    // Kill switch for safety measure
    if (dbus->swl != remote::MID) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    shoot_detector.input(dbus->swr == remote::UP);
    if (shoot_detector.posEdge()) {
      left_fly_motor->SetOutput(1320);
      right_fly_motor->SetOutput(1320);
    } else if (shoot_detector.negEdge()) {
      left_fly_motor->SetOutput(0);
      right_fly_motor->SetOutput(0);
    }

    speed_load = -3 * dbus->ch1 * max_speed / remote::DBUS::ROCKER_MAX;
    speed_yaw = dbus->ch2 * max_speed / remote::DBUS::ROCKER_MAX;
    speed_pitch= dbus->ch3 * max_speed / remote::DBUS::ROCKER_MAX;

    diff_load = load_motor->GetOmegaDelta(speed_load);
    out_load = pid_load.ComputeConstraintedOutput(diff_load);
    load_motor->SetOutput(out_load);

    diff_yaw = yaw_motor->GetOmegaDelta(speed_yaw);
    out_yaw = pid_yaw.ComputeConstraintedOutput(diff_yaw);
    yaw_motor->SetOutput(out_yaw);

    diff_pitch_1 = pitch_motor_1->GetOmegaDelta(speed_pitch);
    out_pitch_1 = pid_pitch_1.ComputeConstraintedOutput(diff_pitch_1);
    pitch_motor_1->SetOutput(out_pitch_1);

    diff_pitch_2 = pitch_motor_2->GetOmegaDelta(speed_pitch);
    out_pitch_2 = pid_pitch_2.ComputeConstraintedOutput(diff_pitch_2);
    pitch_motor_2->SetOutput(out_pitch_2);

    control::MotorCANBase::TransmitOutput(motors, 4);

    osDelay(10);
  }
}
