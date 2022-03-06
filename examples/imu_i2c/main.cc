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
#include "bsp_imu_i2c.h"
#include "cmsis_os.h"
#include "main.h"

static const uint16_t DevAddress = 0x50;
static bsp::IMU *imu = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);
  imu = new bsp::IMU(&hi2c2, DevAddress);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  float angle[3];

  set_cursor(0, 0);
  clear_screen();

  if (!imu->IsRead())
    print("IMU Init Failed!\r\n");

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    imu->GetAngle(angle, true);
    print("Angle: %.3f, %.3f, %.3f\r\n", angle[0], angle[1], angle[2]);
    osDelay(100);
  }
}
