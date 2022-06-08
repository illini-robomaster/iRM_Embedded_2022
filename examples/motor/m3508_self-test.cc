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
#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* motor = nullptr;
static bsp::GPIO *key = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);
  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    if (!key->Read()) {
      int curr[5];
      for (int & i : curr) {
        i = motor->GetCurr();
        osDelay(100);
      }
      bool same = true;
      for (int i = 1; i < 5; ++i) {
        if (curr[0] != curr[i]) {
          same = false;
        }
      }
      print("%s\r\n", same ? "disconnected!!!" : "connected");
    }
    osDelay(100);
  }
}
