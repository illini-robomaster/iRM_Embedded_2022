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
#include "cmsis_os.h"
#include "bsp_imu.h"
#include "pose.h"

static osMutexId_t ahrs_lock;
const osMutexAttr_t ahrs_lock_attr = {
  .name = "ahrs_lock",
  .attr_bits = osMutexRecursive,
  .cb_mem = nullptr,
  .cb_size = 0,
};

static osThreadId_t imu_task_handle;
const osThreadAttr_t imu_task_attr = {
  .name = "imu_task",
  .attr_bits = osThreadDetached,
  .cb_mem = nullptr,
  .cb_size = 0,
  .stack_mem = nullptr,
  .stack_size = 1024 * 4,
  .priority = osPriorityHigh,
  .tz_module = 0,
  .reserved = 0,
};

static bsp::BMI088* bmi088 = nullptr;
static bsp::IST8310* ist8310 = nullptr;
static control::AHRSFilter* filter = nullptr;

void ImuTask(void* argument) {
  UNUSED(argument);

  ist8310 = new bsp::IST8310(&hi2c3, DRDY_IST8310_Pin, bsp::GPIO(GPIOG, GPIO_PIN_6));
  bmi088 = new bsp::BMI088(&hspi1, bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin),
                           bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin), INT1_ACCEL_Pin, INT1_GYRO_Pin);
  filter = new control::AHRSFilter({1, 2, 3},
                                   {1, 2, 3},
                                   {1, 2, 3},
                                   {1, 2, 3},
                                   {1, 2, 3},
                                   {1, 2, 3},
                                   {1, 2, 3},
                                   {1, 2, 3},
                                   ahrs_lock);
}

void RM_RTOS_Mutexes_Init() {
  ahrs_lock = osMutexNew(&ahrs_lock_attr);
}

void RM_RTOS_Threads_Init() {
  imu_task_handle = osThreadNew(ImuTask, nullptr, &imu_task_attr);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  // wait for imu initialization
  osDelay(5000);

  while (true) {
    osDelay(100);
  }
}
