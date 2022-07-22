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
control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::MotorCANBase* motor3 = nullptr;
control::MotorCANBase* motor4 = nullptr;
control::SteeringMotor* steering1 = nullptr;
control::SteeringMotor* steering2 = nullptr;
control::SteeringMotor* steering3 = nullptr;
control::SteeringMotor* steering4 = nullptr;
remote::DBUS* dbus = nullptr;

bsp::GPIO* key = nullptr;

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  // return key->Read() == 1;
  return true;
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim2);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor1 = new control::Motor3508(can1, 0x205);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  //  control::steering_t steering_data;
  //  steering_data.max_speed = SPEED;
  //  steering_data.test_speed = TEST_SPEED;
  //  steering_data.max_acceleration = ACCELERATION;
  //  steering_data.transmission_ratio = 8;
  //  steering_data.offset_angle = 5.96;
  //  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  //  steering_data.max_iout = 1000;
  //  steering_data.max_out = 13000;
  //  steering_data.align_detect_func = steering_align_detect;
  //
  //
  //  steering_data.motor = motor1;
  //  steering1 = new control::SteeringMotor(steering_data);
  //  steering_data.motor = motor2;
  //  steering2 = new control::SteeringMotor(steering_data);
  //  steering_data.motor = motor3;
  //  steering3 = new control::SteeringMotor(steering_data);
  //  steering_data.motor = motor4;
  //  steering4 = new control::SteeringMotor(steering_data);
  //
  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor1, motor2, motor3, motor4};
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  osDelay(500);  // DBUS initialization needs time

  //  print("Alignment Begin\r\n");
  //  while (!steering1->AlignUpdate()) {
  //    control::MotorCANBase::TransmitOutput(motors, 2);
  //    static int i = 0;
  //    if (i > 10) {
  //      steering1->PrintData();
  //      i = 0;
  //    } else {
  //      i++;
  //    }
  //    osDelay(2);
  //  }
  //  print("\r\nAlignment End\r\n");

  float theta0 = 0.0;
  //  double theta1 = 0.0;
  //  double theta2 = 0.0;
  //  double theta3 = 0.0;

  float sign0 = 1.0;

  control::ConstrainedPID pids;
  float* PID_PARAMS = new float[3]{40, 3, 0};
  float MOTOR_MAX_IOUT = 2000;
  float MOTOR_MAX_OUT = 20000;
  pids.Reinit(PID_PARAMS, MOTOR_MAX_IOUT, MOTOR_MAX_OUT);
  float PID_output;
  // power_limit = new PowerLimit(MOTOR_NUM);

  while (true) {
    float vx = static_cast<float>(dbus->ch0) / 660;
    float vy = static_cast<float>(dbus->ch1) / 660;
    float vw = static_cast<float>(dbus->ch2) / 660;

    // kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "operation killed");
    }

    float effort = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vw, 2));

    float theta0_diff;
    //    float theta1_diff;
    //    float theta2_diff;
    //    float theta3_diff;
    if (effort > 0.1) {
      float theta0_new = atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4));
      //      float theta1_new = atan2(vy - vw * cos(PI/4), vx + vw * sin(PI/4));
      //      float theta2_new = atan2(vy + vw * cos(PI/4), vx + vw * sin(PI/4));
      //      float theta3_new = atan2(vy + vw * cos(PI/4), vx - vw * sin(PI/4));
      theta0_diff = wrap<float>(theta0_new - theta0, -PI / 2, PI / 2);
      if (theta0_diff == theta0_new - theta0) {
        sign0 = 1.0;
      } else {
        sign0 = -1.0;
      }
      //      theta1_diff = wrap<float>(wrap<float>(theta1_new - theta1, -PI/2, PI/2), -PI/2, PI/2);
      //      theta2_diff = wrap<float>(wrap<float>(theta2_new - theta2, -PI/2, PI/2), -PI/2, PI/2);
      //      theta3_diff = wrap<float>(wrap<float>(theta3_new - theta3, -PI/2, PI/2), -PI/2, PI/2);
      theta0 = wrap<float>(theta0 + theta0_diff, -PI / 2, PI / 2);
      //      theta1 = wrap<float>(theta1 + theta1_diff, -PI/2, PI/2);
      //      theta2 = wrap<float>(theta2 + theta2_diff, -PI/2, PI/2);
      //      theta3 = wrap<float>(theta3 + theta3_diff, -PI/2, PI/2);
    } else {
      theta0_diff = 0;
      //      theta1_diff = 0;
      //      theta2_diff = 0;
      //      theta3_diff = 0;
    }

    //    steering1->TurnRelative(theta0_diff);
    //    steering2->TurnRelative(theta1_diff);
    //    steering3->TurnRelative(theta2_diff);
    //    steering4->TurnRelative(theta3_diff);
    //    steering1->Update();
    //    steering2->Update();
    //    steering3->Update();
    //    steering4->Update();

    float v0 = sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));

    PID_output = pids.ComputeOutput(motor1->GetOmegaDelta(sign0 * v0 * 10));
    motor1->SetOutput(PID_output);

    control::MotorCANBase::TransmitOutput(motors, 1);
    //    static int i = 0;
    //    if (i > 10) {
    //      print("vx: %10.4f vy: %10.4f vw: %10.4f theta: %10.4f diff: %10.4f\r\n",
    //          vx, vy, vw, theta0, theta0_diff);
    //      i = 0;
    //    } else {
    //      i++;
    //    }

    osDelay(2);
  }
}
