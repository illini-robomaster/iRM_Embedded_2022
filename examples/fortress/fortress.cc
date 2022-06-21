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

#define WITH_CONTROLLER

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#ifdef WITH_CONTROLLER
#include "dbus.h"
#endif

#ifndef WITH_CONTROLLER
#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0
#endif

#define NOTCH (2 * PI / 4)
#define SPEED 200
#define ACCELERATION (80 * PI)

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
control::MotorCANBase* motor_left = nullptr;
control::MotorCANBase* motor_right = nullptr;
control::ServoMotor* servo_left = nullptr;
control::ServoMotor* servo_right = nullptr;
#ifdef WITH_CONTROLLER
remote::DBUS* dbus = nullptr;
#else
BoolEdgeDetector key_detector(false);
#endif

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  // bsp::SetHighresClockTimer(&htim2);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x205, false);

  motor_left = new control::Motor3508(can2, 0x205);
  motor_right = new control::Motor3508(can2, 0x208);

  control::servo_t servo_data;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;

  servo_data.motor = motor_left;
  servo_left = new control::ServoMotor(servo_data);
  servo_data.motor = motor_right;
  servo_right = new control::ServoMotor(servo_data);

#ifdef WITH_CONTROLLER
  dbus = new remote::DBUS(&huart3);
#endif
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
#ifdef WITH_CONTROLLER
  osDelay(500);  // DBUS initialization needs time
#else
  bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
#endif
  control::MotorCANBase* motors[] = {motor_left, motor_right};

  float target = 0;

  while (true) {
#ifdef WITH_CONTROLLER
    target += float(dbus->ch1) / remote::DBUS::ROCKER_MAX / 20;
    servo_left->SetTarget(target, true);
    servo_right->SetTarget(target, true);
#else
    key_detector.input(key.Read() == 0);
    constexpr float desired_target = 5 * 2 * PI;
    if (key_detector.posEdge() && servo_left->Holding() && servo_right->Holding()) {
        servo_left->SetTarget(target);
        servo_right->SetTarget(target);
        target = desired_target - target;
    }
#endif
    servo_left->CalcOutput();
    servo_right->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 2);

    static int i = 0;
    if (i > 15) {
      print("Left:  ");
      servo_left->PrintData();
      print("Right: ");
      servo_right->PrintData();
      i = 0;
    } else {
      i++;
    }

    osDelay(2);
  }
}

// 35.7
