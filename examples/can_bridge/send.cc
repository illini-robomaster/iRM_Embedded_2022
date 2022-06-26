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

#include "bsp_can_bridge.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"

static bsp::CAN* can = nullptr;
static bsp::CanBridge* send = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, 0x401, true);
  send = new bsp::CanBridge(can, 0x401, 0x402);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    send->cmd.vx = -1;
    send->cmd.vy = -10;
    send->cmd.wz = -PI;
    send->TransmitOutput();
    osDelay(1000);
    send->cmd.vx = PI;
    send->cmd.vy = 10;
    send->cmd.wz = 1;
    send->TransmitOutput();
    osDelay(1000);
  }
}
