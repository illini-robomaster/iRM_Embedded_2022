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

#include <Eigen/Dense>

#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_os.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
#include "main.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

typedef struct {
  char header;
  float acce[3];
  float gyro[3];
  float mag[3];
  char terminator;
} __attribute__((packed)) imu_data_t;

static bsp::BMI088* bmi088 = nullptr;
static bsp::IST8310* ist8310 = nullptr;
static bsp::VirtualUSB* usb = nullptr;

static imu_data_t imu_data;

void RM_RTOS_Init(void) {
  bsp::SetHighresClockTimer(&htim1);
  imu_data.header = 's';
  imu_data.terminator = '\0';
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  ist8310 = new bsp::IST8310(&hi2c3, DRDY_IST8310_Pin, bsp::GPIO(GPIOG, GPIO_PIN_6));
  bmi088 = new bsp::BMI088(&hspi1, bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin),
                           bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin),
                           INT1_ACCEL_Pin, INT1_GYRO_Pin);
  usb = new bsp::VirtualUSB();
  usb->SetupTx(sizeof(imu_data_t));
  osDelay(10);

  while (true) {
    taskENTER_CRITICAL();
    memcpy(imu_data.acce, bmi088->accel.data(), 3 * sizeof(float));
    memcpy(imu_data.gyro, bmi088->gyro.data(), 3 * sizeof(float));
    memcpy(imu_data.mag, ist8310->mag.data(), 3 * sizeof(float));
    taskEXIT_CRITICAL();
    usb->Write((uint8_t*)&imu_data, sizeof(imu_data));
    osDelay(10);
  }
}

