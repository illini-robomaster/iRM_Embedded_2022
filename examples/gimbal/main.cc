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

#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"

#define NOTCH (2 * PI / 8)
#define SERVO_SPEED (PI)

bsp::CAN* can = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;

control::Gimbal* gimbal = nullptr;
remote::DBUS* dbus = nullptr;
bool status = false;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1, 0x205);
  pitch_motor = new control::Motor6020(can, 0x205);
  yaw_motor = new control::Motor6020(can, 0x206);

  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal = new control::Gimbal(gimbal_data);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[] = {pitch_motor, yaw_motor};
  control::gimbal_data_t gimbal_data = gimbal->GetData();

  while (true) {
    float pitch_ratio = -dbus->ch3 / 600.0;
    float yaw_ratio = -dbus->ch2 / 600.0;
    if (dbus->swr == remote::UP) {
      gimbal->TargetAbs(pitch_ratio * gimbal_data.pitch_max_, yaw_ratio * gimbal_data.yaw_max_);
    } else if (dbus->swr == remote::MID) {
      gimbal->TargetRel(pitch_ratio / 50, yaw_ratio / 50);
    }

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(10);
  }
}
