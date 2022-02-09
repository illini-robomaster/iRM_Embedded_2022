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
#include "robotic_arm.h"
#include "dbus.h"

#define SPEED (0.5 * PI)
#define ACCELERATION (0.5 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motorL = nullptr;
control::MotorCANBase* motorR = nullptr;
control::MotorCANBase* motorG = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
 print_use_uart(&huart8);
 can1 = new bsp::CAN(&hcan1, 0x201);
 motorL = new control::Motor3508(can1, 0x201);
 motorR = new control::Motor3508(can1, 0x202);
 motorG = new control::Motor3508(can1, 0x203);

// control::servo_t servo_data;
// servo_data.motor = motor;
// servo_data.mode = control::SERVO_NEAREST;
// servo_data.max_speed = SPEED;
// servo_data.max_acceleration = ACCELERATION;
// servo_data.transmission_ratio = M3508P19_RATIO;
// servo_data.omega_pid_param = new float[3]{25, 5, 35};
// servo = new control::ServoMotor(servo_data);

 dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
 UNUSED(args);
 osDelay(500);  // DBUS initialization needs time

 control::robotic_arm_t robotic_arm_data;
 control::MotorCANBase* motors[ARM_MOTOR_NUM] = {motorL, motorR, motorG};
 robotic_arm_data.motors = motors;
 auto* robotic_arm = new control::RoboticArm(robotic_arm_data);
 float target_LR;
 float target_G;

 while (true) {
   target_LR = float(dbus->ch1) / remote::DBUS::ROCKER_MAX * 2 * PI;
   target_G = float(dbus->ch3) / remote::DBUS::ROCKER_MAX * PI;
   robotic_arm->SetPosition(target_LR, target_G);
   robotic_arm->Update();
   control::MotorCANBase::TransmitOutput(motors, ARM_MOTOR_NUM);
   osDelay(10);
 }
}