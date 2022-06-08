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
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    motor->connection_flag_ = false;
    osDelay(50);
    set_cursor(0, 0);
    clear_screen();
    if (motor->connection_flag_) {
      print("Motor Connected...\r\n");
    } else {
      print("Motor Disconnected!!!\r\n");
    }
  }
}
