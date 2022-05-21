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
#include "bsp_imu_i2c.h"
#include "cmsis_os.h"
#include "main.h"

static const uint16_t DevAddress = 0x50;
static bsp::WT901 *imu = nullptr;

void RM_RTOS_Init(void) {
 print_use_uart(&huart8);
 imu = new bsp::WT901(&hi2c2, DevAddress);
}

void RM_RTOS_Default_Task(const void* arguments) {
 UNUSED(arguments);

 float angle[3], acc[3], gyro[3], Q[4];
 int mag[3];

 set_cursor(0, 0);
 clear_screen();

 if (!imu->IsReady())
   print("IMU Init Failed!\r\n");

 while (true) {
   set_cursor(0, 0);
   clear_screen();
   if (!(imu->GetAngle(angle)) || !(imu->GetQuaternion(Q)) || !(imu->GetAcc(acc)) || !(imu->GetGyro(gyro)) || !(imu->GetMag(mag)))
     print("I2C Error!\r\n");
   print("Angle: Pitch = %6.3f, Roll = %6.3f, Yaw = %6.3f\r\n", angle[0], angle[1], angle[2]);
   print("Quaternion: %6.3f, %6.3f, %6.3f, %6.3f\r\n", Q[0], Q[1], Q[2], Q[3]);
   print("\r\n");
   print("Acc: %6.3f, %6.3f, %6.3f\r\n", acc[0], acc[1], acc[2]);
   print("Gyro: %6.3f, %6.3f, %6.3f\r\n", gyro[0], gyro[1], gyro[2]);
   print("Mag: %d, %d, %d\r\n", mag[0], mag[1], mag[2]);
   osDelay(100);
 }
}