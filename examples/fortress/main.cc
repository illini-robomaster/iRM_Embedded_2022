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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "motor.h"
#include "utils.h"

#define NOTCH (2 * PI / 4)
#define SPEED 200
#define ACCELERATION (80 * PI)

static bsp::GPIO* left = nullptr;
static bsp::GPIO* right = nullptr;

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

control::MotorCANBase* motor_left = nullptr;
control::MotorCANBase* motor_right = nullptr;
control::ServoMotor* servo_left = nullptr;
control::ServoMotor* servo_right = nullptr;

BoolEdgeDetector left_edge(true);
BoolEdgeDetector right_edge(true);

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  left = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  right = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);

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
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {motor_left, motor_right};

  float target_left = 0;
  float target_right = 0;

  bool left_reach = false;
  bool right_reach = false;

  while (true) {
    left_edge.input(left->Read());
    right_edge.input(right->Read());

    if (!left_reach && left_edge.negEdge()) {
      target_left = servo_left->GetTheta();
      left_reach = true;
    } else if (!left_reach) {
      target_left -= 0.01;
    }

    if (!right_reach && right_edge.negEdge()) {
      target_right = servo_right->GetTheta();
      right_reach = true;
    } else if (!right_reach) {
      target_right -= 0.01;
    }

    print("Left: %s, Right: %s\r\n", left_reach ? "YES" : "NO", right_reach ? "YES" : "NO");

    servo_left->SetTarget(target_left, true);
    servo_right->SetTarget(target_right, true);
    servo_left->CalcOutput();
    servo_right->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 2);

    if (left_reach && right_reach) break;

    osDelay(2);
  }

  target_left += 35.7;
  target_right += 35.7;

  while (true) {
    servo_left->SetTarget(target_left, true);
    servo_right->SetTarget(target_right, true);
    servo_left->CalcOutput();
    servo_right->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(2);
  }
}
