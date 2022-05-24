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
#include "motor.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

bsp::CAN* can = nullptr;
control::MotorCANBase* motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);

  can = new bsp::CAN(&hcan2, 0x207, false);
  motor = new control::Motor6020(can, 0x207);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};

  bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);
  while (true) {
    if (key.Read()) {
      motor->SetOutput(15000);
    } else {
      motor->SetOutput(15000);
    }
    motor->PrintData();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(100);
  }
}
