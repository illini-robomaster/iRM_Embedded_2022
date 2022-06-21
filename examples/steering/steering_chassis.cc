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
#include "steering.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::MotorCANBase* motor3 = nullptr;
control::MotorCANBase* motor4 = nullptr;
control::MotorCANBase* motor5 = nullptr;
control::MotorCANBase* motor6 = nullptr;
control::MotorCANBase* motor7 = nullptr;
control::MotorCANBase* motor8 = nullptr;

remote::DBUS* dbus = nullptr;

bsp::GPIO* key = nullptr;

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  return key->Read() == 1;
  // return true;
}

// used to init
control::steering_chassis_t* steering_chassis;

control::SteeringChassis* chassis;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim2);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor1 = new control::Motor3508(can1, 0x201);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  motor5 = new control::Motor3508(can1, 0x205);
  motor6 = new control::Motor3508(can1, 0x206);
  motor7 = new control::Motor3508(can1, 0x207);
  motor8 = new control::Motor3508(can1, 0x208);

  steering_chassis = new control::steering_chassis_t();

  steering_chassis->fl_steer_motor = motor1;
  steering_chassis->fr_steer_motor = motor2;
  steering_chassis->bl_steer_motor = motor3;
  steering_chassis->br_steer_motor = motor4;

  steering_chassis->fl_steer_motor_detect_func = steering_align_detect;
  steering_chassis->fr_steer_motor_detect_func = steering_align_detect;
  steering_chassis->bl_steer_motor_detect_func = steering_align_detect;
  steering_chassis->br_steer_motor_detect_func = steering_align_detect;

  // TODO init wheels
  steering_chassis->fl_wheel_motor = motor5;
  steering_chassis->fr_wheel_motor = motor6;
  steering_chassis->bl_wheel_motor = motor7;
  steering_chassis->br_wheel_motor = motor8;

  chassis = new control::SteeringChassis(steering_chassis);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* steer_motors[] = {motor3, motor2, motor3, motor4};
  control::MotorCANBase* wheel_motors[] = {motor7, motor6, motor7, motor8};
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  osDelay(500);  // DBUS initialization needs time

  //  print("Alignment Begin\r\n");
  //  while (!chassis->AlignUpdate()) {
  //      control::MotorCANBase::TransmitOutput(steer_motors, 1);
  //      static int i = 0;
  //      if (i > 10) {
  //        chassis->PrintData();
  //        i = 0;
  //      } else {
  //        i++;
  //      }
  //      osDelay(2);
  //    }
  //  print("\r\nAlignment End\r\n");

  while (true) {
    // kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "operation killed");
    }

    chassis->SetXSpeed(static_cast<float>(dbus->ch0) / 660);
    chassis->SetYSpeed(static_cast<float>(dbus->ch1) / 660);
    chassis->SetWSpeed(static_cast<float>(dbus->ch2) / 660);
    chassis->Update(30, 20, 60);

    control::MotorCANBase::TransmitOutput(wheel_motors, 1);
    control::MotorCANBase::TransmitOutput(steer_motors, 1);

    osDelay(2);
  }
}
