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

#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c3;

//static bsp::BMI088 *imu = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
//  imu = new bsp::BMI088(&hi2c3, DRDY_IST8310_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  bsp::BMI088 imu(&hi2c3, DRDY_IST8310_Pin, GPIOG, GPIO_PIN_6);

  while (true) {
    print("Mag: %.2f, %.2f, %.2f\r\n", imu.mag[0], imu.mag[1], imu.mag[2]);
    osDelay(200);
  }
}
