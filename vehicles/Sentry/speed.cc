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

#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "controller.h"
#include "motor.h"
#include "utils.h"
#include "bsp_gpio.h"

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
static bsp::GPIO* input = nullptr;

const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 256 * 4,
                                          .priority = (osPriority_t)osPriorityNormal,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t chassisTaskHandle;

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase *motors[] = {motor};

  int output = 0;
  int time = 0;
  int direction = 0;
  while (true) {
    direction = rand() % 2 == 1 ? 1 : -1;
    output = direction * (rand() % 9000 + 500);
    time = rand() % 400 + 100;
    int i = 0;
    while (true) {
      if (++i >= time / 2)
        break;
      motor->SetOutput(output);
      control::MotorCANBase::TransmitOutput(motors, 1);
      osDelay(2);
    }
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);
}

void RM_RTOS_Threads_Init(void) {
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  input = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    osDelay(500);
  }
}
