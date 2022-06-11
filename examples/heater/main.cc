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

#include "bsp_heater.h"
#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"

static bsp::Heater *heater = nullptr;
static bsp::BMI088 *BMI088 = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  heater = new bsp::Heater(&htim10, 1, 1000000, 45);
  BMI088 = new bsp::BMI088(&hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  float gyro[3], accel[3], temp;

  int i = 0;
  while (true) {
    BMI088->Read(gyro, accel, &temp);
    float output = heater->Update(temp);
    if (++i % 200 == 0) {
      print("Temperature: %.3f, PWM output: %.3f\r\n", temp, output);
    }
    osDelay(1);
  }
}
