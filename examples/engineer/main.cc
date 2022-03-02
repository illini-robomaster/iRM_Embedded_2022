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
#include "motor.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"

#define gripper_port K2_GPIO_Port
#define gripper_Pin K2_Pin

static bsp::GPIO* gripper;

remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  gripper = new bsp::GPIO(gripper_port, gripper_Pin);
  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);

  while (true) {
    if (dbus->swr == remote::UP) {
      gripper->High();
    } else if (dbus->swr == remote::DOWN) {
      gripper->Low();
    }
    osDelay(10);
  }
}
