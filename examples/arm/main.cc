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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "dbus.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH (2 * PI / 4)
#define SPEED (1 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::ServoMotor* servo1 = nullptr;
control::ServoMotor* servo2 = nullptr;

remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
 print_use_uart(&huart8);
 can1 = new bsp::CAN(&hcan1, 0x201);
 motor1 = new control::Motor3508(can1, 0x201);
 motor2 = new control::Motor3508(can1, 0x202);

 control::servo_t servo1_data;
 servo1_data.motor = motor1;
 servo1_data.mode = control::SERVO_NEAREST;
 servo1_data.speed = SPEED;
 servo1_data.transmission_ratio = M3508P19_RATIO;
 servo1_data.move_Kp = 30;
 servo1_data.move_Ki = 10;
 servo1_data.move_Kd = 40;
 servo1_data.hold_Kp = 3000;
 servo1_data.hold_Ki = 0;
 servo1_data.hold_Kd = 1000;
 servo1 = new control::ServoMotor(servo1_data, PI);

 control::servo_t servo2_data;
 servo2_data.motor = motor2;
 servo2_data.mode = control::SERVO_NEAREST;
 servo2_data.speed = SPEED;
 servo2_data.transmission_ratio = M3508P19_RATIO;
 servo2_data.move_Kp = 3;
 servo2_data.move_Ki = 1;
 servo2_data.move_Kd = 4;
 servo2_data.hold_Kp = 1500;
 servo2_data.hold_Ki = 15;
 servo2_data.hold_Kd = 100;
 servo2 = new control::ServoMotor(servo2_data);

 dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
 UNUSED(args);
 osDelay(500);  // DBUS initialization needs time

 control::MotorCANBase* motors[] = {motor1, motor2};

 float target1 = 0;
 float target2 = 0;

 while (true) {
   // joystick input range from -660 to 660
   target1 = (dbus->ch1 + 20 > 0 ? dbus->ch1 + 20 : 0) * PI / 660;
   target2 = (dbus->ch3 > 0 ? dbus->ch3 : 0) * PI / 660;

   servo1->SetTarget(target1);
   servo2->SetTarget(target2);

   servo1->CalcOutput();
   servo2->CalcOutput();

   servo1->PrintData();
   servo2->PrintData();
   control::MotorCANBase::TransmitOutput(motors, 2);
   osDelay(10);
 }
}
