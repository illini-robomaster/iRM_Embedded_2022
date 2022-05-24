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
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "motor.h"
#include "fortress.h"

static bsp::CAN* can = nullptr;

static control::MotorCANBase* fortress_motor0 = nullptr;
static control::MotorCANBase* fortress_motor1 = nullptr;
static control::MotorCANBase* fortress_yaw_motor = nullptr;
static control::Fortress* fortress = nullptr;

static remote::DBUS* dbus = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);
  dbus = new remote::DBUS(&huart1);
  can = new bsp::CAN(&hcan2, 0x205, true);
  fortress_motor0 = new control::Motor3508(can, 0x205);
  fortress_motor1 = new control::Motor3508(can, 0x208);
  control::MotorCANBase* fortress_motors[FORTRESS_MOTOR_NUM];
  fortress_motors[0] = fortress_motor0;
  fortress_motors[1] = fortress_motor1;
  fortress_yaw_motor = new control::Motor6020(can, 0x207);
  fortress = new control::Fortress(fortress_motors, fortress_yaw_motor);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  osDelay(500);

  control::MotorCANBase* motors_fortress[] = {fortress_motor0, fortress_motor1, fortress_yaw_motor};

  int transform_time = 1000;

  while (dbus->swr != remote::DOWN) {
    if (dbus->swr == remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  while (dbus->swr == remote::DOWN) {
    if (dbus->swr != remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  while (true) {
    if (dbus->swr == remote::DOWN) {
      for (int i = 0; i < transform_time / 10; ++i) {
        fortress->Up();
        fortress->StopRotate();
        control::MotorCANBase::TransmitOutput(motors_fortress, FORTRESS_MOTOR_NUM + 1);
        osDelay(10);
      }
      while (dbus->swr == remote::DOWN) {
        fortress->StopRise();
        fortress->Rotate();
        control::MotorCANBase::TransmitOutput(motors_fortress, FORTRESS_MOTOR_NUM + 1);
        osDelay(10);
      }
    } else {
      for (int i = 0; i < transform_time / 10; ++i) {
        fortress->Down();
        fortress->StopRotate();
        control::MotorCANBase::TransmitOutput(motors_fortress, FORTRESS_MOTOR_NUM + 1);
        osDelay(10);
      }
      while (dbus->swr != remote::DOWN) {
        fortress->StopRise();
        fortress->StopRotate();
        control::MotorCANBase::TransmitOutput(motors_fortress, FORTRESS_MOTOR_NUM + 1);
        osDelay(10);
      }
    }
  }
}
