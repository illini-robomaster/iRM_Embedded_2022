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

#include <cstring>
#include "main.h"

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "bsp_imu_i2c.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "shooter.h"
#include "chassis.h"

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
control::MotorCANBase* sl_motor = nullptr;
control::MotorCANBase* sr_motor = nullptr;
control::MotorCANBase* ld_motor = nullptr;
control::Shooter* shooter = nullptr;
remote::DBUS* dbus = nullptr;
static const uint16_t IMUAddress = 0x50;
static bsp::IMU *imu = nullptr;
static bsp::GPIO *gpio_red, *gpio_green;

void RM_RTOS_Init() {
  print_use_uart(&huart8);

  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

  imu = new bsp::IMU(&hi2c2, IMUAddress);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);

  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can2, 0x206);

  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_STANDARD_2022_ALPHA;
  gimbal = new control::Gimbal(gimbal_data);

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

  sl_motor = new control::Motor3508(can1, 0x202);
  sr_motor = new control::Motor3508(can1, 0x203);
  ld_motor = new control::Motor3508(can1, 0x201);
  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = sl_motor;
  shooter_data.right_flywheel_motor = sr_motor;
  shooter_data.load_motor = ld_motor;
  shooter = new control::Shooter(shooter_data);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors_can1_pitch[] = {pitch_motor};
  control::MotorCANBase* motors_can2_yaw[] = {yaw_motor};
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

control::gimbal_data_t gimbal_data = gimbal->GetData();

  if (!imu->IsRead())
    RM_ASSERT_TRUE(false, "IMU Init Failed!\r\n");

 float angle[3], angle_read[3], angle_offset[3];
 float pitch_target = 0, yaw_target = 0;
 float pitch_curr, yaw_curr;

  print("Calirate\r\n");
  gpio_red->Low();
  gpio_green->Low();
  for (int i = 0; i < 1000; i++) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
    osDelay(5);
  }
  gpio_red->High();
  gpio_green->High();

  if (!(imu->GetAngle(angle_offset)))
    RM_ASSERT_TRUE(false, "I2C Error!\r\n");
  print("Begin\r\n");
  imu->GetAngle(angle);

  while (true) {
   if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed\r\n");
      fl_motor->SetOutput(0);
      bl_motor->SetOutput(0);
      fr_motor->SetOutput(0);
      br_motor->SetOutput(0);
      pitch_motor->SetOutput(0);
      yaw_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
      control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
      control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);
      control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
   }
     
    if (!(imu->GetAngle(angle_read)))
      RM_ASSERT_TRUE(false, "I2C Error!\r\n");

    // print("%8.6f, %8.6f, %8.6f |", angle[0] / PI * 180, angle[1] / PI * 180, angle[2] / PI * 180);

    angle_read[0] = wrap<float>(angle_read[0] - angle_offset[0], -PI, PI);
    angle_read[1] = wrap<float>(angle_read[1] - angle_offset[1], -PI, PI);
    angle_read[2] = wrap<float>(angle_read[2] - angle_offset[2], -PI, PI);

    float alpha = -0.15;
    angle[0] = angle[0] * alpha + angle_read[0] * (1 - alpha);
    angle[1] = angle[1] * alpha + angle_read[1] * (1 - alpha);
    angle[2] = angle[2] * alpha + angle_read[2] * (1 - alpha);

    if (dbus->swr == remote::UP) {
      float pitch_ratio = dbus->ch3 / 600.0;
      float yaw_ratio = -dbus->ch2 / 600.0;
      pitch_target = clip<float>(pitch_target + pitch_ratio / 40.0, 
          -gimbal_data.pitch_max_, gimbal_data.pitch_max_);
      yaw_target = wrap<float>(yaw_target + yaw_ratio / 30.0, -PI, PI);
    }
    pitch_curr = -angle[1];
    yaw_curr = angle[2];
    float pitch_diff = wrap<float>(pitch_target - pitch_curr, -PI, PI);
    float yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);
    
    float yaw_offset = 0;
    if (dbus->swr == remote::MID) {
      chassis->SetSpeed(-dbus->ch0 / 2, -dbus->ch1 / 2, dbus->ch2 / 2);
      yaw_offset = dbus->ch2 / 660.0 * 2 * PI / 6 / 3;
    }

    gimbal->TargetRel(pitch_diff / 18, (yaw_diff + yaw_offset) / 30);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

    chassis->Update();
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);

    // UNUSED(pitch_diff);
    // UNUSED(yaw_diff);
    // UNUSED(yaw_offset);
    // UNUSED(motors_can2_chassis);
    // UNUSED(motors_can1_shooter);

    // yaw_motor->SetOutput(2000);
    // control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
    // yaw_motor->PrintData();

    osDelay(5);
  }
}


//  if (!imu->IsRead())
//    RM_ASSERT_TRUE(false, "IMU Init Failed!\r\n");

//  float angle[3];
//  float pitch_target = 0, yaw_target = 0;
//  float pitch_curr, yaw_curr;

//  while (true) {
//    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN)
//      RM_ASSERT_TRUE(false, "Operation killed\r\n");

//    if (!(imu->GetAngle(angle)))
//      RM_ASSERT_TRUE(false, "I2C Error!\r\n");

//    if (dbus->swr == remote::UP) {
//      float pitch_ratio = dbus->ch3 / 600.0;
//      float yaw_ratio = -dbus->ch2 / 600.0;
//      pitch_target = wrap<float>(pitch_target + pitch_ratio / 40.0, -PI, PI);
//      yaw_target = wrap<float>(yaw_target + yaw_ratio / 30.0, -PI, PI);
//    }
//    pitch_curr = -angle[1];
//    yaw_curr = angle[2];
//    float yaw_diff;
//    if (-PI < yaw_target && yaw_target < -PI / 2 && PI / 2 < yaw_curr && yaw_curr < PI) {
//      yaw_diff = yaw_target - yaw_curr + 2 * PI;
//    } else if (-PI < yaw_curr && yaw_curr < -PI / 2 && PI / 2 < yaw_target && yaw_target < PI) {
//      yaw_diff = yaw_target - yaw_curr - 2 * PI;
//    } else {
//      yaw_diff = yaw_target - yaw_curr;
//    }
   
//    float yaw_offset = 0;
//    if (dbus->swr == remote::MID) {
//      chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
//      yaw_offset = dbus->ch2 / 660.0 * 2 * PI / 6;
//    }

//    gimbal->TargetRel((pitch_target - pitch_curr) / 18, (yaw_diff + yaw_offset) / 30);
//    gimbal->Update();
//    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
//    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

//    chassis->Update();
//    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);
   