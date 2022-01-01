/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2020 RoboMaster.                                          *
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

control::MotorPWMBase* motor1;
control::MotorPWMBase* motor2;


void RM_RTOS_Init() {
  print_use_uart(&huart8);
  motor1 = new control::MotorPWMBase(&htim1, 1, 1000000, 500, 1080);
  motor2 = new control::MotorPWMBase(&htim1, 4, 1000000, 500, 1080);
  motor1->SetOutput(0);
  motor2->SetOutput(0);
  osDelay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);
  motor2->SetOutput(5);

  while (1) {
    if (key.Read()) {
      motor1->SetOutput(300);
      motor2->SetOutput(300);
    } else {
      motor1->SetOutput(0);
      motor2->SetOutput(0);
    }
    osDelay(1000);
  }
}
