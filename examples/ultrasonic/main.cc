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
#include "bsp_ultrasonic.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

bsp::Ultrasonic* ultrasonic;

void RM_RTOS_Init(void) {
	print_use_uart(&huart8);
  HAL_TIM_Base_Start_IT(&htim2);
  ultrasonic = new bsp::Ultrasonic(ULTRASONIC_Trig_GPIO_Port,
                                   ULTRASONIC_Trig_Pin,
                                   ULTRASONIC_Echo_GPIO_Port,
                                   ULTRASONIC_Echo_Pin,
                                   TIM2);
}

void RM_RTOS_Default_Task(const void* arguments) {
	UNUSED(arguments);

  while (true) {
    float distance = ultrasonic->GetDistance();
    if (distance > 0) {
      set_cursor(0, 0);
      clear_screen();
      print("Distance: %.2f cm\r\n", distance);
    }
    osDelay(50);
  }
}
