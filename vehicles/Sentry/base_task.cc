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

#include <cstring>
#include <memory>

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "lidar07.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH (2 * PI / 4)
#define SPEED 200
#define ACCELERATION (80 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;

BoolEdgeDetector key_detector(false);

// remote::DBUS* dbus = nullptr;

bsp::GPIO* gpio_red;

const osThreadAttr_t baseTaskAttribute = {.name = "baseTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 512 * 4,
                                          .priority = (osPriority_t)osPriorityRealtime,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t baseTaskHandle;
// declare for the base Task
void baseTask(void* argument);
static distance::LIDAR07_UART* LIDAR = nullptr;

void RM_RTOS_Threads_Init(void) {
  baseTaskHandle = osThreadNew(baseTask, nullptr, &baseTaskAttribute);
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);
  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_red->High();

  // print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201);

  motor = new control::Motor3508(can1, 0x201);
  LIDAR = new distance::LIDAR07_UART(&huart6, [](uint32_t milli) { osDelay(milli); });

  // dbus = new remote::DBUS(&huart3);
}

// dummy main thread to test multi-tasks
void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time
  while (true) {
    gpio_red->Toggle();
    osDelay(500);
  }
}

void baseTask(void* argument) {
  UNUSED(argument);
  osDelay(500);  // DBUS initialization needs time
  print("something is working here");
  while (LIDAR->begin()) osDelay(50);
  print("Begin\r\n");
  while (LIDAR->startFilter()) osDelay(50);
  print("Start Filter\r\n");

  //control::MotorCANBase* motors[] = {motor};
  float direction = 1.0;
//  const float MAX = 2.0 * PI;
//  const float MIN = -2.0 * PI;
  //float step = PI;
  int a = 0;
  int b = 2000;
  float pos = (rand()%(b-a+1))+ a;
  if (LIDAR->distance > pos) {
    direction = -1.0;
    motor->SetOutput(-50);
  } else {
    direction = 1.0;
    motor->SetOutput(50);
  }
  while (true) {
    while (!LIDAR->startMeasure()) osDelay(50);
    print("theta: % .2f m\r\n ", LIDAR->distance/1000.0);
    //print("in the loop");
    if (LIDAR->distance > pos - 100 || LIDAR->distance < pos + 100) {
      // You can reset the position can reset the target;
      if (direction > 0) {
        a = 0;
        b = LIDAR->distance;
      } else {
        a = LIDAR->distance;
        b = 2000;
      }
      direction = -direction;
      if (direction > 0) motor->SetOutput(50);
      else motor->SetOutput(-50);
      pos = (rand()%(b-a+1))+ a;
    }
    //print("hey");
    //control::MotorCANBase::TransmitOutput(motors, 1);
    //print("can we get here");
    osDelay(100);
  }
}
