/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2021 RoboMaster.                                          *
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
#include "cmsis_os.h"

int32_t pub = 0;

osMutexId_t addLockHandle;

const osMutexAttr_t addLock = {
  .name = "addLock",
  .attr_bits = osMutexRecursive,
  .cb_mem = NULL,
  .cb_size = 0
};

/* init new task START */
osThreadId_t ADD_TaskHandle;

const osThreadAttr_t AddTask_attributes = {
  .name = "AddTask",
  .attr_bits = osThreadDetached,
  .cb_mem = NULL,
  .cb_size = 0,
  .stack_mem = NULL,
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
  .tz_module = 0,
  .reserved = 0
};

void AddTask(void *argument) {
  UNUSED(argument);
  osStatus_t ret;
  while (1) {
    ret = osMutexAcquire(addLockHandle, osWaitForever);
    if(ret == osOK) {
      pub += 1;
      print("%d, by thread 2\r\n", pub);
    }
    osMutexRelease(addLockHandle);
    osDelay(2000);
  }
}

/* init new task END */

void RM_RTOS_Mutexes_Init(void) {
  addLockHandle =	osMutexNew (&addLock);
}

void RM_RTOS_Threads_Init(void) {
  ADD_TaskHandle = osThreadNew(AddTask, NULL, &AddTask_attributes);
}


void RM_RTOS_Init(void) {

}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osStatus_t ret;
  print_use_uart(&huart8);
  while (1) {
    ret = osMutexAcquire(addLockHandle, osWaitForever);
    if(ret == osOK) {  
      pub += 1;
      print("%d, by thread 1\r\n", pub);
    }
    osMutexRelease(addLockHandle);
    osDelay(3000);
  }
}

