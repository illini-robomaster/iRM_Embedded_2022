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
#define ACCELERATION (50 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::SteeringMotor* steering = nullptr;
remote::DBUS* dbus = nullptr;

bsp::GPIO* key = nullptr;

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  return key->Read() == 1;
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim2);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);

  control::steering_t steering_data;
  steering_data.motor = motor;
  steering_data.max_speed = SPEED;
  steering_data.test_speed = TEST_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = 8;
  steering_data.offset_angle = 5.96;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.align_detect_func = steering_align_detect;
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

  double theta = 0.0;

  while (true) {
    float vx = static_cast<float>(dbus->ch0) / 660;
    float vy = static_cast<float>(dbus->ch1) / 660;
    float vw = static_cast<float>(dbus->ch2) / 660;

    // kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "operation killed");
    }
    
    float effort = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vw, 2));

    float theta_diff;
    if (effort > 0.1) {
      float theta_new = atan2(vy - vw * cos(PI/4), vx - vw * sin(PI/4));
      theta_diff = wrap<float>(wrap<float>(theta_new - theta, -PI/2, PI/2), -PI/2, PI/2);
      theta = wrap<float>(theta + theta_diff, -PI/2, PI/2);
    } else {
      theta_diff = 0;
    }

    steering->TurnRelative(theta_diff);
    steering->Update();
    control::MotorCANBase::TransmitOutput(motors, 1);

    static int i = 0;
    if (i > 10) {
      print("vx: %10.4f vy: %10.4f vw: %10.4f theta: %10.4f diff: %10.4f\r\n", 
          vx, vy, vw, theta, theta_diff);
      i = 0;
    } else {
      i++;
    }

    osDelay(2);
  }
}
