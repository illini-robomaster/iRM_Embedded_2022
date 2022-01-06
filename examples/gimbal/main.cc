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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "gimbal.h"
#include "shooter.h"
#include "dbus.h"


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH         (2 * PI / 8)
#define SERVO_SPEED   (PI)

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
  gimbal_data.pitch_offset = LEGACY_GIMBAL_POFF;
  gimbal_data.yaw_offset = LEGACY_GIMBAL_YOFF;
  gimbal_data.pitch_max = LEGACY_GIMBAL_PMAX;
  gimbal_data.yaw_max = LEGACY_GIMBAL_YMAX;
  gimbal_data.pitch_proximity = LEGACY_GIMBAL_PMAX / 3;
  gimbal_data.yaw_proximity = LEGACY_GIMBAL_YMAX / 6;
  gimbal_data.pitch_move_Kp = 800;
  gimbal_data.pitch_move_Ki = 0;
  gimbal_data.pitch_move_Kd = 100;
  gimbal_data.yaw_move_Kp = 300;
  gimbal_data.yaw_move_Ki = 0;
  gimbal_data.yaw_move_Kd = 100;
  gimbal_data.pitch_hold_Kp = 2000;
  gimbal_data.pitch_hold_Ki = 100;
  gimbal_data.pitch_hold_Kd = 100;
  gimbal_data.yaw_hold_Kp = 1500;
  gimbal_data.yaw_hold_Ki = 15;
  gimbal_data.yaw_hold_Kd = 200;
  gimbal = new control::Gimbal(gimbal_data);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

	osDelay(500); // DBUS initialization needs time
  
  control::MotorCANBase* motors[] = {pitch_motor, yaw_motor};

  while (true) {
    float pitch_ratio = -dbus->ch3 / 600.0;
    float yaw_ratio = -dbus->ch2 / 600.0;
    if (dbus->swr == remote::UP) {
      gimbal->TargetAbs(pitch_ratio * LEGACY_GIMBAL_PMAX, yaw_ratio * LEGACY_GIMBAL_YMAX);
    } else if (dbus->swr == remote::MID) {
      gimbal->TargetRel(pitch_ratio / 50, yaw_ratio / 50);
    } 
    
    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    gimbal->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(10);
  }
}
