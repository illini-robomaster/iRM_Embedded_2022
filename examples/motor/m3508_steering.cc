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
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define SPEED (10 * PI)
#define TEST_SPEED (0.5 * PI)
#define ACCELERATION (100 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::SteeringMotor* steering = nullptr;
remote::DBUS* dbus = nullptr;

bsp::GPIO* key = nullptr;

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  return key->Read() == 1;
  // return true;
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim2);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x202);

  control::steering_t steering_data;
  steering_data.motor = motor;
  steering_data.max_speed = SPEED;
  steering_data.test_speed = TEST_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.offset_angle = 5.36;
  steering_data.omega_pid_param = new float[3]{100, 0.5, 12};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.align_detect_func = steering_align_detect;
  // steering_data.calibrate_offset = 0.858458848;
  steering_data.calibrate_offset = 0;
  steering = new control::SteeringMotor(steering_data);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  osDelay(500);  // DBUS initialization needs time

  print("Alignment Begin\r\n");
  while (!steering->AlignUpdate()) {
    control::MotorCANBase::TransmitOutput(motors, 1);
    static int i = 0;
    if (i > 10) {
      steering->PrintData();
      i = 0;
    } else {
      i++;
    }
    osDelay(2);
  }
  print("\r\nAlignment End\r\n");

  while (true) {
    // steering->TurnRelative(float(dbus->ch1) / remote::DBUS::ROCKER_MAX / 20);
    steering->Update();
    control::MotorCANBase::TransmitOutput(motors, 1);

    static int i = 0;
    if (i > 10) {
      steering->PrintData();
      i = 0;
    } else {
      i++;
    }

    osDelay(2);
  }
}
