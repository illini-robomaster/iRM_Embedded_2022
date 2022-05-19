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

void kill_all() {
  control::MotorCANBase* motors_can1_pitch[] = {pitch_motor};
  control::MotorCANBase* motors_can2_yaw[] = {yaw_motor};
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

  fl_motor->SetOutput(0);
  bl_motor->SetOutput(0);
  fr_motor->SetOutput(0);
  br_motor->SetOutput(0);
  pitch_motor->SetOutput(0);
  yaw_motor->SetOutput(0);
  sl_motor->SetOutput(0);
  sr_motor->SetOutput(0);
  ld_motor->SetOutput(0);
  control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
  control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
  control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);
  control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
  RM_ASSERT_TRUE(false, "Operation killed\r\n");
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

  float angle[3];
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;

  print("Calirate\r\n");
  gpio_red->Low();
  gpio_green->Low();
  for (int i = 0; i < 400; i++) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
    if (dbus->swl == remote::DOWN) 
      kill_all();
    osDelay(5);
  }
  gpio_red->High();
  gpio_green->High();

  if (!(imu->SetAngleOffset()))
    RM_ASSERT_TRUE(false, "I2C Error!\r\n");
  print("Begin\r\n");

  float sin_yaw, cos_yaw;
  float vx, vy, wz;
  float vx_set, vy_set, wz_set;
  float relative_angle;

  FirstOrderFilter f0(0.3);
  FirstOrderFilter f1(0.3);
  FirstOrderFilter f2(0.3);
  FirstOrderFilter f3(0.3);

  while (true) {
    float ch0 = f0.CalculateOutput(dbus->ch0);
    float ch1 = f1.CalculateOutput(dbus->ch1);
    float ch2 = f2.CalculateOutput(dbus->ch2);
    float ch3 = f3.CalculateOutput(dbus->ch3);

    if (dbus->swl == remote::DOWN) 
      kill_all();
     
    if (!(imu->GetAngle(angle)))
      RM_ASSERT_TRUE(false, "I2C Error!\r\n");

    // print("%8.5f, %8.5f, %8.5f |", angle[0] / PI * 180, angle[1] / PI * 180, angle[2] / PI * 180);

    float yaw_offset = 0;

    if (dbus->swr == remote::MID) {
      float pitch_ratio = ch3 / 600.0;
      float yaw_ratio = -ch2 / 600.0;
      pitch_target = clip<float>(pitch_target + pitch_ratio / 40.0, 
          -gimbal_data.pitch_max_, gimbal_data.pitch_max_);

      // float current_diff = wrap<float>(yaw_target + yaw_motor->GetThetaDelta(gimbal_data.yaw_offset_), -PI, PI);
      // print("%10.6f\r\n", current_diff);
      // float added_diff = current_diff + yaw_ratio / 30.0;
      // if (added_diff < PI && added_diff > -PI)
      yaw_target = wrap<float>(yaw_target + yaw_ratio / 30.0, -PI, PI);
      
      vx = ch0;
      vy = ch1;
      UNUSED(wz);
      relative_angle = yaw_motor->GetThetaDelta(gimbal_data.yaw_offset_);
      sin_yaw = arm_sin_f32(relative_angle);
      cos_yaw = arm_cos_f32(relative_angle);
      vx_set = cos_yaw * vx + sin_yaw * vy;
      vy_set = -sin_yaw * vx + cos_yaw * vy;
      if (dbus->swl == remote::UP) {
        wz_set = 250;
      } else {
        wz_set = 250 * relative_angle;
      }
      wz_set = clip<float>(wz_set, -290, 290);
      chassis->SetSpeed(vx_set, vy_set, wz_set);
    }

    pitch_curr = -angle[1];
    yaw_curr = angle[2];
    float pitch_diff = wrap<float>(pitch_target - pitch_curr, -PI, PI);
    float yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);
    // print("%8.5f, %8.5f\r\n", pitch_diff, yaw_diff);

    gimbal->TargetRel(pitch_diff / 28, (yaw_diff + yaw_offset) / 30);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

    chassis->Update();
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);

    osDelay(5);
  }
}
