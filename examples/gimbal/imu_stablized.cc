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
#include "bsp_laser.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "main.h"
#include "pose.h"

// init gimbal
#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
remote::DBUS* dbus = nullptr;
bsp::GPIO *gpio_red, *gpio_green;

// init imu
#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

static bsp::MPU6500* imu;
static control::Pose* poseEstimator;

/* init IMU task START */

/*
static osMutexId_t rpyLockHandle;

const osMutexAttr_t rpyLock = {
  .name = "rpyLock",
  .attr_bits = osMutexRecursive,
  .cb_mem = nullptr,
  .cb_size = 0
};
*/

static osThreadId_t IMUTaskHandle;

const osThreadAttr_t IMUTaskAttributes = {.name = "AddTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 128 * 4,
                                          .priority = (osPriority_t)osPriorityNormal,
                                          .tz_module = 0,
                                          .reserved = 0};

// pose estimation task
void IMU_Task(void* argument) {
  UNUSED(argument);

  // IMU init
  gpio_green->High();
  gpio_red->High();
  bsp::GPIO chip_select(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
  imu = new bsp::MPU6500(&ONBOARD_IMU_SPI, chip_select, MPU6500_IT_Pin);
  osDelay(4000);
  gpio_green->Low();
  print("IMU Initialized!\r\n");

  // init pose estimator instance
  poseEstimator = new control::Pose(imu);

  // Set alpha for the complementary filter in the pose estimator
  poseEstimator->SetAlpha(0.99);

  // calibrate the Offset for IMU acce meter and gyro
  // Need the gimbal to be stable for 1sec
  poseEstimator->Calibrate();

  // reset timer and pose for IMU
  poseEstimator->PoseInit();
  gpio_red->Low();

  while (true) {
    // update estimated pose with complementary filter
    poseEstimator->ComplementaryFilterUpdate();
    osDelay(2);
  }
} /* IMU task ends */

/* init IMU task END */

/*
void RM_RTOS_Mutexes_Init(void) {
  rpyLockHandle = osMutexNew (&rpyLock);
}
*/
void RM_RTOS_Threads_Init(void) {
  IMUTaskHandle = osThreadNew(IMU_Task, nullptr, &IMUTaskAttributes);
}

void RM_RTOS_Init() {
  print_use_uart(&PRING_UART);

  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

  bsp::SetHighresClockTimer(&htim2);
  can1 = new bsp::CAN(&hcan1, 0x205, true);
  can2 = new bsp::CAN(&hcan2, 0x205, false);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);

  control::gimbal_t gimbal_data;
  gimbal_data.model = control::GIMBAL_STANDARD_2022_ALPHA;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal = new control::Gimbal(gimbal_data);

  dbus = new remote::DBUS(&huart1);
}

// Gimbal task
void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  // control::MotorCANBase* motors[] = {pitch_motor, yaw_motor};
  control::MotorCANBase* motors[] = {yaw_motor};

  float yaw, pitch;  //, roll;
  int i = 0;

  osDelay(5000);
  gpio_green->High();
  gpio_red->High();
  float yaw_offset = yaw_motor->GetTheta();

  while (true) {
    // update rpy
    yaw = poseEstimator->GetYaw();
    pitch = poseEstimator->GetPitch();
    // roll = poseEstimator->GetRoll();
    gimbal->TargetRel(pitch / 8, -yaw / 33);

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 1);

    i += 1;
    if (i >= 50) {
      set_cursor(0, 0);
      clear_screen();
      yaw_motor->PrintData();
      print("Y_DEG: %6.4f \r\n", yaw / PI * 180);
      i = 0;
    }

    osDelay(2);
  }
}
