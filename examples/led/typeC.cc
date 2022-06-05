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

static bsp::GPIO *gpio_red, *gpio_green, *gpio_blue;

void RM_RTOS_Init(void) {
  gpio_red = new bsp::GPIO(LED_R_GPIO_Port, LED_R_Pin);
  gpio_green = new bsp::GPIO(LED_G_GPIO_Port, LED_G_Pin);
  gpio_blue = new bsp::GPIO(LED_B_GPIO_Port, LED_B_Pin);
//  gpio_red->Low();
//  gpio_green->Low();
//  gpio_blue->Low();
//  gpio_red->High();
//  gpio_green->High();
//  gpio_blue->High();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  while (true) {
    gpio_red->High();
    gpio_green->Low();
    gpio_blue->Low();
    osDelay(500);
    gpio_red->Low();
    gpio_green->High();
    gpio_blue->Low();
    osDelay(500);
    gpio_red->Low();
    gpio_green->Low();
    gpio_blue->High();
    osDelay(500);
  }
}
