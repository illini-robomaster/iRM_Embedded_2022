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
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "main.h"

// gimbalTask() task handler and attribute
extern const osThreadAttr_t gimbalTaskAttribute;
osThreadId_t gimbalTaskHandle;

// gimbalTask() declaration
extern void gimbalTask(void* arg);

void RM_RTOS_Threads_Init(void) {
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}

void RM_RTOS_Init(void) {}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  while (1) {
  }
}
