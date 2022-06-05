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

#include <cstring>

#include "bsp_gpio.h"
#include "bsp_imu_i2c.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "main.h"
#include "shooter.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

control::MotorCANBase* yaw_motor = nullptr;

static bsp::GPIO *gpio_red, *gpio_green;

void RM_RTOS_Init() {
  print_use_uart(&huart8);

  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

  can2 = new bsp::CAN(&hcan2, 0x206, false);

  yaw_motor = new control::Motor6020(can2, 0x206);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* can2_yaw[] = {yaw_motor};

  while (true) {
    float omega = yaw_motor->GetOmega();
    int sign = omega < 0 ? -1 : 1;
    float yaw_offset = 0;
    if (omega < -0.01 || omega > 0.01) yaw_offset = omega * 968.2362 + sign * 713.84;
    print("%8.4f\r\n", omega);
    yaw_motor->SetOutput(yaw_offset);
    control::MotorCANBase::TransmitOutput(can2_yaw, 1);
    osDelay(3);
  }

  int wait_len = 400;
  int sample_len = 200;

  float speed_vec_start = -5000;
  float speed_vec_interval = -500;
  int speed_vec_len = 20;
  float* inputs = new float[speed_vec_len];
  float* outputs = new float[speed_vec_len];
  for (int i = 0; i < speed_vec_len; i++) {
    inputs[i] = speed_vec_start + i * speed_vec_interval;
    outputs[i] = 0;
    for (int j = 0; j < wait_len; j++) {
      yaw_motor->SetOutput(inputs[i]);
      control::MotorCANBase::TransmitOutput(can2_yaw, 1);
      osDelay(5);
    }
    for (int j = 0; j < sample_len; j++) {
      yaw_motor->SetOutput(inputs[i]);
      outputs[i] += yaw_motor->GetOmega();
      control::MotorCANBase::TransmitOutput(can2_yaw, 1);
      osDelay(5);
    }
    outputs[i] /= sample_len;
    print("%i Complete\r\n", i);
  }

  while (true) {
    print("x = [");
    for (int i = 0; i < speed_vec_len - 1; i++) print("%6.3f, ", outputs[i]);
    print("%6.3f]\r\n", outputs[speed_vec_len - 1]);

    print("y = [");
    for (int i = 0; i < speed_vec_len - 1; i++) print("%6.3f, ", inputs[i]);
    print("%6.3f]\r\n", inputs[speed_vec_len - 1]);

    osDelay(5000);
  }

  osDelay(5);
}

//  if (!imu->IsRead())
//    RM_ASSERT_TRUE(false, "IMU Init Failed!\r\n");

//  float angle[3];
//  float pitch_target = 0, yaw_target = 0;
//  float pitch_curr, yaw_curr;

//  while (true) {
//    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN)
//      RM_ASSERT_TRUE(false, "Operation killed\r\n");

//    if (!(imu->GetAngle(angle)))
//      RM_ASSERT_TRUE(false, "I2C Error!\r\n");

//    if (dbus->swr == remote::UP) {
//      float pitch_ratio = dbus->ch3 / 600.0;
//      float yaw_ratio = -dbus->ch2 / 600.0;
//      pitch_target = wrap<float>(pitch_target + pitch_ratio / 40.0, -PI, PI);
//      yaw_target = wrap<float>(yaw_target + yaw_ratio / 30.0, -PI, PI);
//    }
//    pitch_curr = -angle[1];
//    yaw_curr = angle[2];
//    float yaw_diff;
//    if (-PI < yaw_target && yaw_target < -PI / 2 && PI / 2 < yaw_curr && yaw_curr < PI) {
//      yaw_diff = yaw_target - yaw_curr + 2 * PI;
//    } else if (-PI < yaw_curr && yaw_curr < -PI / 2 && PI / 2 < yaw_target && yaw_target < PI) {
//      yaw_diff = yaw_target - yaw_curr - 2 * PI;
//    } else {
//      yaw_diff = yaw_target - yaw_curr;
//    }

//    float yaw_offset = 0;
//    if (dbus->swr == remote::MID) {
//      chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
//      yaw_offset = dbus->ch2 / 660.0 * 2 * PI / 6;
//    }

//    gimbal->TargetRel((pitch_target - pitch_curr) / 18, (yaw_diff + yaw_offset) / 30);
//    gimbal->Update();
//    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
//    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

//    chassis->Update();
//    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);
