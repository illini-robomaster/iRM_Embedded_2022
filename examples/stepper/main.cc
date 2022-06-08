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
#include "bsp_gpio.h"
#include "cmsis_os.h"

bsp::GPIO *key = nullptr, *dir = nullptr, *pul = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);
  dir = new bsp::GPIO(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin);
  pul = new bsp::GPIO(STEPPER_PUL_GPIO_Port, STEPPER_PUL_Pin);
  pul->Low();
}

//void Delay_us(uint16_t us) {
//  uint32_t ticks = 0;
//  uint32_t told = 0, tnow = 0, tcnt = 0;
//  uint32_t reload = 0;
//  reload = SysTick->LOAD;
//  ticks = us * 72;
//  told = SysTick->VAL;
//  while (true)
//  {
//    tnow = SysTick->VAL;
//    if (tnow != told)
//    {
//      if (tnow < told)
//      {
//        tcnt += told - tnow;
//      }
//      else
//      {
//        tcnt += reload - tnow + told;
//      }
//      told = tnow;
//      if (tcnt >= ticks)
//      {
//        break;
//      }
//    }
//  }
//}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  int step = 4000;
  int period = 1;
  bool direction = false;
  while (true) {
    if (!key->Read()) {
      direction = !direction;
      if (direction) {
        dir->High();
      } else {
        dir->Low();
      }
      for (int i = 0; i < step; ++i) {
        pul->High();
        osDelay(period);
        pul->Low();
        osDelay(period);
      }
    }
    osDelay(50);
  }
}
