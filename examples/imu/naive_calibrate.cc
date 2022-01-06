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

#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_os.h"
#include "cmsis_os.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

static bsp::MPU6500* imu;

void RM_RTOS_Init(void) {
	bsp::SetHighresClockTimer(&htim2);
	print_use_uart(&PRING_UART);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  bsp::GPIO chip_select(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
  imu = new bsp::MPU6500(&ONBOARD_IMU_SPI, chip_select, MPU6500_IT_Pin);

  print("IMU Initialized!\r\n");
  
  // ignore the first 3s
  osDelay(3000);
  
  float acce_x = 0;
  float acce_y = 0;
  float acce_z = 0;
  float gyro_x = 0;
  float gyro_y = 0;
  float gyro_z = 0;
  
  const uint16_t NUM = 100;
  
  // average NUM times acc and gyro reads
  for (int i = 0; i < NUM; i++) {
    acce_x += imu->acce.x;
    acce_y += imu->acce.y;
    acce_z += imu->acce.z;
    gyro_x += imu->gyro.x;
    gyro_y += imu->gyro.y;
    gyro_z += imu->gyro.z;
    osDelay(10);
  }
  
  acce_x /= (float) NUM;
  acce_y /= (float) NUM;
  acce_z /= (float) NUM;
  gyro_x /= (float) NUM;
  gyro_y /= (float) NUM;
  gyro_z /= (float) NUM;
  
  set_cursor(0, 0);
  clear_screen();
  print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", acce_x, acce_y, acce_z);
  print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", gyro_x, gyro_y, gyro_z);
  
  while (true) {
  }
}
