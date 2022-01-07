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

#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "main.h"

static bsp::GPIO *gpio_red, *gpio_green;

/* init new task START */
static osThreadId_t LED_GREEN_TaskHandle;

const osThreadAttr_t LED2Task_attributes = {
  .name = "LEDGreenTask",
  .attr_bits = osThreadDetached,
  .cb_mem = nullptr,
  .cb_size = 0,
  .stack_mem = nullptr,
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
  .tz_module = 0,
  .reserved = 0
};

void LED_GREEN_Task(void *argument) {
  UNUSED(argument);
  while (true) {
    gpio_green->Toggle();
    osDelay(200);
  }
}
/* init new task END */


void RM_RTOS_Threads_Init(void) {
  LED_GREEN_TaskHandle = osThreadNew(LED_GREEN_Task, nullptr, &LED2Task_attributes);
}


void RM_RTOS_Init(void) {
  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  gpio_red->High();
  gpio_green->Low();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  while (true) {
    gpio_red->Toggle();
    osDelay(500);
  }
}

