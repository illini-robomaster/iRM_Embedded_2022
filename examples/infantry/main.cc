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
#include "bsp_print.h"
#include "bsp_uart.h"
#include "controller.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "chassis.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;
control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);

  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can2, 0x206);

  fl_motor = new control::Motor3508(can2, 0x201);
  fr_motor = new control::Motor3508(can2, 0x202);
  bl_motor = new control::Motor3508(can2, 0x203);
  br_motor = new control::Motor3508(can2, 0x204);
  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;

  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_STANDARD_ZERO;
  chassis = new control::Chassis(chassis_data);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

  float sin_yaw = 0, cos_yaw = 0;
  float offset = 5.246;
  float relative_angle = 0;
  float vx, vy, wz;
  float vx_set, vy_set, wz_set;

  control::PIDController rotate_pid;
  float* pid_param = new float[3]{100, 0, 0};
  rotate_pid.Reinit(pid_param);

  while (true) {
   if (dbus->swl == remote::DOWN)
     RM_ASSERT_TRUE(false, "Operation killed\r\n");

   if (dbus->swr == remote::MID) {
     vx = dbus->ch0;
     vy = dbus->ch1;
     wz = dbus->ch2;
     relative_angle = yaw_motor->GetThetaDelta(offset);
     sin_yaw = arm_sin_f32(relative_angle);
     cos_yaw = arm_cos_f32(relative_angle);
     vx_set = cos_yaw * vx + sin_yaw * vy;
     vy_set = -sin_yaw * vx + cos_yaw * vy;
     wz_set = rotate_pid.ComputeOutput(relative_angle);
//     if (-PI / 2 < relative_angle && relative_angle < PI / 2) {
//       wz_set = 0;
//     }
     UNUSED(wz);
     UNUSED(wz_set);

     if (dbus->swl == remote::UP) {
       wz_set = 80;
     } else {
       wz_set = 0;
     }
     chassis->SetSpeed(vx_set, vy_set, wz_set);
   }

   print("%.6f\r\n", yaw_motor->GetTheta());

    chassis->Update();
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

    osDelay(5);
  }
}
   