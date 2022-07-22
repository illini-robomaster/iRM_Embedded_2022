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

#include <cmath>

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH (2 * PI / 4)
#define SPEED (4 * PI)
#define ACCELERATION (8 * PI)
#define TEST_SPEED (0.5 * PI)

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
bsp::GPIO* key = nullptr;

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  return key->Read() == 1;
}

control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::MotorCANBase* motor3 = nullptr;
control::MotorCANBase* motor4 = nullptr;
control::MotorCANBase* motor5 = nullptr;
control::MotorCANBase* motor6 = nullptr;
control::MotorCANBase* motor7 = nullptr;
control::MotorCANBase* motor8 = nullptr;

typedef struct {
  control::SteeringMotor* fl_steer_motor = nullptr;
  control::SteeringMotor* fr_steer_motor = nullptr;
  control::SteeringMotor* bl_steer_motor = nullptr;
  control::SteeringMotor* br_steer_motor = nullptr;

  control::MotorCANBase* fl_wheel_motor = nullptr;
  control::MotorCANBase* fr_wheel_motor = nullptr;
  control::MotorCANBase* bl_wheel_motor = nullptr;
  control::MotorCANBase* br_wheel_motor = nullptr;

  double radius = 1.0;

} steering_chassis_t;
steering_chassis_t* chassis = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart6);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  // servo
  control::steering_t steering_data;

  steering_data.max_speed = SPEED;
  steering_data.test_speed = TEST_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = 8;
  steering_data.offset_angle = 5.96;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.align_detect_func = steering_align_detect;

  can1 = new bsp::CAN(&hcan1, 0x201, false);
  can2 = new bsp::CAN(&hcan2, 0x205, true);

  motor1 = new control::Motor3508(can1, 0x201);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  steering_data.motor = motor1;
  chassis->fl_steer_motor = new control::SteeringMotor(steering_data);

  steering_data.motor = motor2;
  chassis->fr_steer_motor = new control::SteeringMotor(steering_data);

  steering_data.motor = motor3;
  chassis->bl_steer_motor = new control::SteeringMotor(steering_data);

  steering_data.motor = motor4;
  chassis->br_steer_motor = new control::SteeringMotor(steering_data);

  // wheel
  chassis->fl_wheel_motor = new control::Motor3508(can2, 0x205);
  chassis->fr_wheel_motor = new control::Motor3508(can2, 0x206);
  chassis->bl_wheel_motor = new control::Motor3508(can2, 0x207);
  chassis->br_wheel_motor = new control::Motor3508(can2, 0x208);

  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  double theta0 = 0.0;
  double theta1 = 0.0;
  double theta2 = 0.0;
  double theta3 = 0.0;

  while (true) {
    double vx = static_cast<double>(dbus->ch0) / 660;
    double vy = static_cast<double>(dbus->ch1) / 660;
    double vw = static_cast<double>(dbus->ch2) / 660;

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    //    double v0 = sqrt(pow(vy - vw * cos(PI/4), 2.0) + pow(vx - vw * sin(PI/4), 2.0));
    //    double v1 = sqrt(pow(vy - vw * cos(PI/4), 2.0) + pow(vx + vw * sin(PI/4), 2.0));
    //    double v2 = sqrt(pow(vy + vw * cos(PI/4), 2.0) + pow(vx + vw * sin(PI/4), 2.0));
    //    double v3 = sqrt(pow(vy + vw * cos(PI/4), 2.0) + pow(vx - vw * sin(PI/4), 2.0));

    double _theta0 = atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4));
    double _theta1 = atan2(vy - vw * cos(PI / 4), vx + vw * sin(PI / 4));
    double _theta2 = atan2(vy + vw * cos(PI / 4), vx + vw * sin(PI / 4));
    double _theta3 = atan2(vy + vw * cos(PI / 4), vx - vw * sin(PI / 4));

    // update with relative angles (-180 - 180)
    chassis->fl_steer_motor->TurnRelative(clip<double>(_theta0 - theta0, -PI, PI));
    chassis->fr_steer_motor->TurnRelative(clip<double>(_theta1 - theta1, -PI, PI));
    chassis->bl_steer_motor->TurnRelative(clip<double>(_theta2 - theta2, -PI, PI));
    chassis->br_steer_motor->TurnRelative(clip<double>(_theta3 - theta3, -PI, PI));
    theta0 = _theta0;
    theta1 = _theta1;
    theta2 = _theta2;
    theta3 = _theta3;

    osDelay(10);
  }
  // 1: -2.283133461
  // 2: 2.283133461
  // 3: 0.858458848
  // 4: -0.858458848
}
