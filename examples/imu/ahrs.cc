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
#include "bsp_os.h"
#include "bsp_usb.h"
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
  .stack_size = 4096 * 4,
  .priority = osPriorityHigh,
  .tz_module = 0,
  .reserved = 0,
};

static osThreadId_t print_task_handle;
const osThreadAttr_t print_task_attr = {
  .name = "print_task",
  .attr_bits = osThreadDetached,
  .cb_mem = nullptr,
  .cb_size = 0,
  .stack_mem = nullptr,
  .stack_size = 1024 * 4,
  .priority = osPriorityLow,
  .tz_module = 0,
  .reserved = 0,
};

static bsp::BMI088* bmi088 = nullptr;
static bsp::IST8310* ist8310 = nullptr;
static control::AHRSFilter* filter = nullptr;
static bsp::VirtualUSB* usb = nullptr;

#define ACCEL_INT_FLAG  (1 << 0)
#define GYRO_INT_FLAG   (1 << 1)
#define MAG_INT_FLAG    (1 << 2)

static void AccelCallback(const bsp::BMI088& bmi088) {
  UNUSED(bmi088);
  osThreadFlagsSet(imu_task_handle, ACCEL_INT_FLAG);
}

static void GyroCallback(const bsp::BMI088& bmi088) {
  UNUSED(bmi088);
  osThreadFlagsSet(imu_task_handle, GYRO_INT_FLAG);
}

static void MagCallback(const bsp::IST8310& ist8310) {
  UNUSED(ist8310);
  osThreadFlagsSet(imu_task_handle, MAG_INT_FLAG);
}

void ImuTask(void* argument) {
  UNUSED(argument);

  // wait for usb initialization
  osDelay(1000);

  ist8310 = new bsp::IST8310(&hi2c3, DRDY_IST8310_Pin, bsp::GPIO(GPIOG, GPIO_PIN_6));
  bmi088 = new bsp::BMI088(&hspi1, bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin),
                           bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin), INT1_ACCEL_Pin, INT1_GYRO_Pin);
  osDelay(20); // wait for imu init

  // compute average fields
  //Eigen::Vector3f g = Eigen::Vector3f::Zero();
  //Eigen::Vector3f m = Eigen::Vector3f::Zero();
  //const uint8_t cnt = 200;
  //for (uint32_t i = 0; i < cnt; ++i) {
  //  g = g + bmi088->accel;
  //  m = m + ist8310->mag;
  //  osDelay(10);
  //}
  //g = g / cnt;
  //m = m / cnt;

  filter = new control::AHRSFilter({1e-5, 1e-5, 1e-5},
                                   {1.6e-5, 1.7e-5, 1.9e-5},
                                   {0.07, 0.13, 0.9},
                                   {1e-7, 1e-7, 1e-7},
                                   {9, 9, 9},
                                   {400, 400, 400},
                                   {0, 0, -1},
                                   {22.5428, 5.2346, 41.6421},
                                   ahrs_lock);


  // initialize filter
  filter->Initialize(Eigen::Quaternionf::Identity(),
                     Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Ones() * 0.1,
                     Eigen::Vector3f::Ones() * 4e-4,
                     Eigen::Vector3f::Ones() * 3e-4,
                     Eigen::Vector3f::Ones() * 1e3,
                     bmi088->gyro,
                     bmi088->gyro_timestamp);
  bmi088->RegisterAccelCallback(AccelCallback);
  bmi088->RegisterGyroCallback(GyroCallback);
  ist8310->RegisterCallback(MagCallback);

  while (true) {
    uint32_t flags = osThreadFlagsWait(ACCEL_INT_FLAG | GYRO_INT_FLAG | MAG_INT_FLAG,
                                       osFlagsWaitAny, osWaitForever);
    uint32_t filter_timestamp = filter->GetLatestTime();
    if ((flags & ACCEL_INT_FLAG) && bmi088->accel_timestamp > filter_timestamp) {
      filter->MeasureAccel(bmi088->accel, bmi088->accel_timestamp);
    }
    if ((flags & GYRO_INT_FLAG) && bmi088->gyro_timestamp > filter_timestamp) {
      filter->MeasureGyro(bmi088->gyro, bmi088->gyro_timestamp);
    }
    if ((flags & MAG_INT_FLAG) && ist8310->timestamp > filter_timestamp) {
      filter->MeasureMag(ist8310->mag, ist8310->timestamp);
    }
  }
}

typedef struct {
  float quat[4];
  float accel_bias[3];
  float gyro_bias[3];
  float mag_bias[3];
} __attribute__((packed)) ahrs_data_t;

void PrintTask(void* argument) {
  UNUSED(argument);
  print_use_uart(&huart1);
  usb = new bsp::VirtualUSB;
  usb->SetupTx(2048);

  // wait for imu initialization
  osDelay(5000);

  ahrs_data_t packet;

  while (true) {
    Eigen::Quaternionf pose;
    Eigen::Vector3f ab, gb, mb;
    filter->GetLatestState(&pose, &ab, &gb, &mb);

    set_cursor(0, 0);
    clear_screen();
    print("pose %.6f %.6f %.6f %.6f\r\n", pose.x(), pose.y(), pose.z(), pose.w());
    print("accel bias %.6f %.6f %.6f\r\n", ab.x(), ab.y(), ab.z());
    print("gyro bias %.6f %.6f %.6f\r\n", gb.x(), gb.y(), gb.z());
    print("mag bias %.6f %.6f %.6f\r\n", mb.x(), mb.y(), mb.z());
    memcpy(packet.quat, pose.coeffs().data(), sizeof(float) * 4);
    memcpy(packet.accel_bias, ab.data(), sizeof(float) * 3);
    memcpy(packet.gyro_bias, gb.data(), sizeof(float) * 3);
    memcpy(packet.mag_bias, mb.data(), sizeof(float) * 3);

    usb->Write((uint8_t*)&packet, sizeof(ahrs_data_t));
    osDelay(100);
  }
}

void RM_RTOS_Init(void) {
  bsp::SetHighresClockTimer(&htim5);
}

void RM_RTOS_Mutexes_Init() {
  ahrs_lock = osMutexNew(&ahrs_lock_attr);
}

void RM_RTOS_Threads_Init() {
  imu_task_handle = osThreadNew(ImuTask, nullptr, &imu_task_attr);
  print_task_handle = osThreadNew(PrintTask, nullptr, &print_task_attr);
}
