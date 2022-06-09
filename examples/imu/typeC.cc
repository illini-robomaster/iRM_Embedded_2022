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
#include "i2c.h"

#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"

static bsp::IST8310 *IST8310 = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  IST8310 = new bsp::IST8310(&hi2c3, DRDY_IST8310_Pin, GPIOG, GPIO_PIN_6);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  print("%s\r\n", IST8310->IsReady() ? "Ready" : "Not Ready");
  osDelay(500);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("Mag: %.2f, %.2f, %.2f\r\n", IST8310->mag[0], IST8310->mag[1], IST8310->mag[2]);
    osDelay(100);
  }
}
