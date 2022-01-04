/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2020 RoboMaster.                                          *
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
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "gimbal.h"


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH   (2 * PI / 8)
#define SERVO_SPEED   (PI)

bsp::CAN* can = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::MotorCANBase* pluck_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::ServoMotor* pluck = nullptr;
BoolEdgeDetecter key_detecter(false);
bool status = false;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1, 0x205);
  pitch_motor = new control::Motor6020(can, 0x205);
  yaw_motor = new control::Motor6020(can, 0x206);
  pluck_motor = new control::Motor2006(can, 0x207);

  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.pitch_offset = LEGACY_GIMBAL_POFF;
  gimbal_data.yaw_offset = LEGACY_GIMBAL_YOFF;
  gimbal_data.pitch_Kp = 10;
  gimbal_data.pitch_Ki = 0.25;
  gimbal_data.pitch_Kd = 0.15;
  gimbal_data.yaw_Kp = 10;
  gimbal_data.yaw_Ki = 0.15;
  gimbal_data.yaw_Kd = 0.15;
  gimbal = new control::Gimbal(gimbal_data);
  
  control::servo_t servo_data;
  servo_data.motor = pluck_motor;
  servo_data.mode = SERVO_ANTICLOCKWISE;
  servo_data.speed = SERVO_SPEED;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.move_Kp = 20;
  servo_data.move_Ki = 15;
  servo_data.move_Kd = 30;
  servo_data.hold_Kp = 40;
  servo_data.hold_Ki = 15;
  servo_data.hold_Kd = 5;
  pluck = new control::ServoMotor(servo_data); 
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {pitch_motor, yaw_motor, pluck_motor};
  bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);

  float target = 0;

  while (true) {
    key_detecter.input(key.Read());
    if (key_detecter.posEdge() && pluck->SetTarget(target) != 0) {
      target = wrap<float>(target + NOTCH, -PI, PI);
    }
    gimbal->CalcOutput();
    pluck->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 3);
    osDelay(10);
  }
}
