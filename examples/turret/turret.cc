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

/**
 * @brief This example is intended to run on official legacy gimbal. Please read the instructions below to 
 * avoid possible danger.
 * 
 * Controller configuration
 * 
 *  shooting motor on UP                                                  UP auto-shooting mode
 * shooting motor off MID    Left Switch                 Right Switch    MID single shot mode
 *        kill switch DOWN                                              DOWN not implemented
 *                Left Joystick                                   Right Joystick
 *       increase pitch ^                                                 ^ shoot
 *      decrease yaw <     > increase yaw   change gimbal control mode <     > change gimbal control mode
 *       decrease pitch v                                                 v not implemented
 * 
 * <-------------------------------------------!!! WARNING !!!------------------------------------------->
 * DO <NOT> STAND BY THE GIMBAL WHEN IT IS POWERED, MAKE SURE KILL SWITCH IS ACTIVATED BEFORE APPROACHING
 * 
 * @date 2022-01-05
 */

#define GIMBAL_2019

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "gimbal.h"
#include "shooter.h"
#include "dbus.h"


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH               (2 * PI / 8)
#define SERVO_SPEED         (2 * PI)
#define GIMBAL_SPEED        PI
#define JOYSTICK_THRESHOLD  (remote::DBUS::ROCKER_MAX * 4 / 5)

bsp::CAN* can = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::MotorPWMBase* left_fly_motor = nullptr;
control::MotorPWMBase* right_fly_motor = nullptr;
control::MotorCANBase* load_motor = nullptr;
BoolEdgeDetector shoot_detector(false);
BoolEdgeDetector load_detector(false);
BoolEdgeDetector abs_detector(false);

control::ServoMotor* load_servo = nullptr;

control::Gimbal* gimbal = nullptr;
control::Shooter* shooter = nullptr;
remote::DBUS* dbus = nullptr;
bool status = false;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1, 0x205);
  pitch_motor = new control::Motor6020(can, 0x205);
  yaw_motor = new control::Motor6020(can, 0x206);
  // See pwm example
  left_fly_motor = new control::MotorPWMBase(&htim1, 1, 1000000, 500, 1080);
  right_fly_motor = new control::MotorPWMBase(&htim1, 4, 1000000, 500, 1080);
  load_motor = new control::Motor2006(can, 0x207);

  control::servo_t servo_data;
  servo_data.motor = load_motor;
  servo_data.mode = control::SERVO_ANTICLOCKWISE;
  servo_data.speed = SERVO_SPEED;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.move_Kp = 20;
  servo_data.move_Ki = 15;
  servo_data.move_Kd = 30;
  servo_data.hold_Kp = 40;
  servo_data.hold_Ki = 15;
  servo_data.hold_Kd = 5;
  load_servo = new control::ServoMotor(servo_data); 

  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal = new control::Gimbal(gimbal_data);
  
  control::shooter_t shooter_data;
  shooter_data.fly_using_can_motor = false;
  shooter_data.left_fly_pwm_motor = left_fly_motor;
  shooter_data.right_fly_pwm_motor = right_fly_motor;
  shooter_data.load_servo = load_servo;
  shooter_data.fly_Kp = 80;
  shooter_data.fly_Ki = 3;
  shooter_data.fly_Kd = 0.1;
  shooter_data.load_step_angle = 2 * PI / 8;
  shooter = new control::Shooter(shooter_data);  

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {pitch_motor, yaw_motor, load_motor};
	bsp::GPIO laser(LASER_GPIO_Port, LASER_Pin);
	laser.High();

  bool load = false;
  bool abs_mode = true;

	osDelay(500); // DBUS initialization needs time

  while (true) {
    // Kill switch for safety measure
    if (dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    // Toggle gimbal control absolute or relative mode
    // 
    //    To toggle push right joystick left or right to the end
    abs_detector.input(dbus->ch0 <= -JOYSTICK_THRESHOLD || dbus->ch0 >= JOYSTICK_THRESHOLD);
    if (abs_detector.posEdge()) {
      abs_mode = !abs_mode;
    }
    float pitch_ratio = -dbus->ch3 / remote::DBUS::ROCKER_MAX;
    float yaw_ratio = -dbus->ch2 / remote::DBUS::ROCKER_MAX;
    if (abs_mode) {
      gimbal->TargetAbs(pitch_ratio * GIMBAL_PITCH_MAX, yaw_ratio * GIMBAL_YAW_MAX);
    } else {
      // divide by 100 since osDelay is 10
      gimbal->TargetRel(pitch_ratio * GIMBAL_SPEED / 100, yaw_ratio * GIMBAL_SPEED / 100);
    }

    // Toggle load control contigious or single shot on right switch
    //    Up for contiguous load
    //    Mid for load per right joystick pushed up
    bool load_trigger = dbus->ch1 >= JOYSTICK_THRESHOLD;
    load_detector.input(load_trigger);
    if (dbus->swr == remote::UP) {
      load = load_trigger;
    } else if (dbus->swr == remote::MID) {
      load = load_detector.posEdge();
    }
    if (load)
      shooter->LoadNext();

    // Toggle shoot status on or off on left switch
    //    Up for shoot motor start
    //    Mid for shoot motor stop
    shoot_detector.input(dbus->swl == remote::UP);
    if (shoot_detector.posEdge()) {
      shooter->SetFlywheelSpeed(150);
    } else if (shoot_detector.negEdge()) {
      shooter->SetFlywheelSpeed(0);
    } 

    // Update and send command
    gimbal->Update();
    shooter->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 3);
    osDelay(10);
  }
}
