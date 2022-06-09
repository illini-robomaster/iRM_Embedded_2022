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
#include "oled.h"

static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* motor = nullptr;
static display::OLED *OLED = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);
  OLED = new display::OLED(&hi2c2, 0x3C);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  print("OLED %s\r\n", OLED->IsReady() ? "Ready" : "Not Ready");
  OLED->ShowLOGO();
  osDelay(200);
  OLED->OperateGram(display::PEN_CLEAR);
  OLED->ShowString(0, 0, (uint8_t*)"C1");

  while (true) {
    motor->connection_flag_ = false;
    osDelay(50);
    set_cursor(0, 0);
    clear_screen();
    if (motor->connection_flag_) {
      OLED->ShowBlock(0, 2, true);
      print("Motor Connected...\r\n");
    } else {
      OLED->ShowBlock(0, 2, false);
      print("Motor Disconnected!!!\r\n");
    }
    OLED->RefreshGram();
  }
}
