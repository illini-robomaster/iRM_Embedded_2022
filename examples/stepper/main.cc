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
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "cmsis_os.h"

bsp::PWM stepper(&htim1, 1, 1000000, 0, 0);
bsp::GPIO *key = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);
  stepper.Start();
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  int freq = 200;
  while (true) {
    if (key->Read()) {
      stepper.SetFrequency(200);
      stepper.SetPulseWidth(1000000/ freq / 2);
    } else {
      stepper.SetFrequency(0);
      stepper.SetPulseWidth(1000000/ freq / 2);
    }
  }
}
