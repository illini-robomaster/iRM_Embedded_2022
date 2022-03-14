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
#include "bsp_laser.h"
#include "bsp_imu_i2c.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "main.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;
control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

static const uint16_t IMUAddress = 0x50;
static bsp::IMU *imu = nullptr;

void RM_RTOS_Init() {
 print_use_uart(&huart8);
 imu = new bsp::IMU(&hi2c2, IMUAddress);
 can1 = new bsp::CAN(&hcan1, 0x205, true);
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

 control::gimbal_t gimbal_data;
 gimbal_data.pitch_motor = pitch_motor;
 gimbal_data.yaw_motor = yaw_motor;
 gimbal_data.model = control::GIMBAL_STANDARD_2022_ALPHA;
 gimbal = new control::Gimbal(gimbal_data);

 dbus = new remote::DBUS(&huart1);
}

// Gimbal task
void RM_RTOS_Default_Task(const void* args) {
 UNUSED(args);

 osDelay(500);  // DBUS initialization needs time

 control::MotorCANBase* motors_can1_pitch[] = {pitch_motor};
 control::MotorCANBase* motors_can2_yaw[] = {yaw_motor};
 control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

 if (!imu->IsRead())
   RM_ASSERT_TRUE(false, "IMU Init Failed!\r\n");

 float angle[3];
 float pitch_ = 0, yaw_ = 0;

 while (true) {
   // Kill switch
   if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
     RM_ASSERT_TRUE(false, "Operation killed");
   }

//   if (!(imu->GetAngle(angle)))
//     print("I2C Error!\r\n");
//   gimbal->TargetRel(-angle[0] / 8, -angle[2] / 30);
//   gimbal->Update();
//   control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
//   control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
//
//   osDelay(10);
//
//   if (dbus->swr == remote::UP) {
//     float pitch_ratio = dbus->ch3 / 600.0;
//     float yaw_ratio = -dbus->ch2 / 600.0;
//     gimbal->TargetRel(pitch_ratio / 30, yaw_ratio / 30);
//     gimbal->Update();
//   }
//   control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
//   control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

//   float pitch_, yaw_;
//   if (!(imu->GetAngle(angle)))
//     print("I2C Error!\r\n");
//   pitch_ = -angle[0] / 8;
//   yaw_ = -angle[2] / 30;
//
//   if (dbus->swr == remote::UP) {
//     float pitch_ratio = dbus->ch3 / 600.0;
//     float yaw_ratio = -dbus->ch2 / 600.0;
//     pitch_ += pitch_ratio / 30;
//     yaw_ += yaw_ratio / 30;
//   }
//   gimbal->TargetRel(pitch_, yaw_);
//   gimbal->Update();
//   control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
//   control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

   if (!(imu->GetAngle(angle)))
     print("I2C Error!\r\n");

   if (dbus->swr == remote::UP) {
     float pitch_ratio = dbus->ch3 / 600.0;
     float yaw_ratio = -dbus->ch2 / 600.0;
     pitch_ += pitch_ratio / 3000.0;
     yaw_ += yaw_ratio / 3000.0;
   }
   gimbal->TargetRel(pitch_ - angle[0] / 8, yaw_ - angle[2] / 30);
   gimbal->Update();
   control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
   control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

   if (dbus->swr == remote::MID)
     chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
   chassis->Update();
   control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

   osDelay(10);
 }
}
