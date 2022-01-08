
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

bsp::Ultrasonic* ultrasonic1;
bsp::Ultrasonic* ultrasonic2;

void RM_RTOS_Init(void) {
	print_use_uart(&huart8);
  HAL_TIM_Base_Start_IT(&htim2);
  ultrasonic1 = new bsp::Ultrasonic(ULTRASONIC1_Trig_GPIO_Port,
                                   ULTRASONIC1_Trig_Pin,
                                   ULTRASONIC1_Echo_GPIO_Port,
                                   ULTRASONIC1_Echo_Pin,
                                   TIM2);
	ultrasonic2 = new bsp::Ultrasonic(ULTRASONIC2_Trig_GPIO_Port,
	                                  ULTRASONIC2_Trig_Pin,
	                                  ULTRASONIC2_Echo_GPIO_Port,
	                                  ULTRASONIC2_Echo_Pin,
	                                  TIM2);


}

void RM_RTOS_Default_Task(const void* arguments) {
	UNUSED(arguments);

  while (true) {
    float distance1 = ultrasonic1->GetDistance();
		float distance2 = ultrasonic2->GetDistance();
    set_cursor(0, 0);
    clear_screen();
    if (distance1 > 0 && distance2 > 0) {
      print("Distance1: %.2f cm   Distance2:  %.2f cm\r\n", distance1, distance2);
    } else if (distance1 > 0 && distance2 < 0) {
	    print("Distance1: %.2f cm   ERROR!!!\r\n", distance1);
		} else if (distance2 > 0 && distance1 < 0) {
	    print("ERROR!!!   Distance2: %.2f cm\r\n", distance2);
    } else {
      print("BOTH ERROR!!!\r\n");
    }
    osDelay(20);
  }
}
