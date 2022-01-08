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

#include <cmath>

#include "main.h"

#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_os.h"
#include "cmsis_os.h"
#include "bsp_print.h"

#include "pose.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

#define DEG2RAD(x) ((x) / 180 * M_PI)
#define MPU6500_ACC_FACTOR 4096.0f
#define MPU6500_GYRO_FACTOR 32.768f
#define USEC_TO_SEC 1000000.0f

static bsp::MPU6500* imu;

void RM_RTOS_Init(void) {
	bsp::SetHighresClockTimer(&htim2);
	print_use_uart(&PRING_UART);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  bsp::GPIO chip_select(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
  imu = new bsp::MPU6500(&ONBOARD_IMU_SPI, chip_select, MPU6500_IT_Pin);

  //control::Pose estimator(imu);
  //bsp::vec3f_t *pose = new bsp::vec3f_t{0, 0, 0};

  print("IMU Initialized!\r\n");
  osDelay(3000);
  
  // measured with naive_calibrate.cc
  const float ACC_X_OFF = 0.0394;
  const float ACC_Y_OFF = -0.0892;
  const float ACC_Z_OFF = -0.0806;
  /*
  const float GYRO_X_OFF = -0.0285;
  const float GYRO_Y_OFF = 0.0110;
  const float GYRO_Z_OFF = 0.0115;
  */
  //float yaw = 0;
  float roll = 0;
  float pitch = 0;
  float a = 0.95;
  float alpha = a;
  float pitchAcc = 0;
  float rollAcc = 0;
  uint32_t ts = imu->timestamp;
  //uint32_t curr_ts = 0;
  
  while (true) {
    
    pitchAcc = atan2f(imu->acce.y - ACC_Y_OFF, imu->acce.z - ACC_Z_OFF);
    rollAcc = atan2f(imu->acce.x - ACC_X_OFF, imu->acce.z - ACC_Z_OFF);
    
    // 
    // RAD 1.3 and 1.4 are measured values where pitch / roll error might be > 0.1
    if (abs(rollAcc) > 1.0 || abs(pitchAcc) > 1.0) {
      alpha = 1.0;
    } else {
      alpha = a;
    }
    
    pitch = alpha * (pitch + (imu->gyro.x) * (float)(imu->timestamp - ts) / USEC_TO_SEC) + (1.0 - alpha) * pitchAcc;
    roll = alpha * (roll + (imu->gyro.y) * (float)(imu->timestamp - ts) / USEC_TO_SEC) - (1.0 - alpha) * rollAcc;
    ts = imu->timestamp;

    osDelay(10);

    set_cursor(0, 0);
    clear_screen();
    print("ALPHA: %1.2f \r\n", alpha);
    print("PITCH: %8.4f ROLL: %8.4f \r\n", pitchAcc, rollAcc);
    print("PITCH: %8.4f ROLL: %8.4f \r\n", pitch, roll);
  }
}
