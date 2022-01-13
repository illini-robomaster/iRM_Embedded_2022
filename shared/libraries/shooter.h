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

#pragma once

#include "motor.h"
#include "controller.h"

namespace control {

/**
 * @brief structure used when shooter instance is initialized
 */
typedef struct {
  bool fly_using_can_motor;					 /* if flywheel motors are using CAN protocal   */
  MotorCANBase* left_fly_can_motor;	 /* CAN motor instance of left flywheel motor   */
  MotorCANBase* right_fly_can_motor; /* CAN motor instance of right flywheel motor  */
  MotorPWMBase* left_fly_pwm_motor;	 /* PWM motor instance of left flywheel motor   */
  MotorPWMBase* right_fly_pwm_motor; /* PWM motor instance of right flywheel motor  */
  bool left_fly_motor_invert;				 /* if left flywheel motor is inverted          */
  bool right_fly_motor_invert;			 /* if right flywheel motor is inverted         */
  ServoMotor* load_servo;            /* servomotor instance of load motor           */
  float fly_Kp;                      /* Kp of pid controlling flywheel motor speed  */
  float fly_Ki;                      /* Ki of pid controlling flywheel motor speed  */
  float fly_Kd;                      /* Kd of pid controlling flywheel motor speed  */
  float load_step_angle;             /* step size of loading motor, in [rad]        */
} shooter_t;

/**
 * @brief wrapper class for shooter
 */
class Shooter {
public:
  /**
   * @brief constructor for Shooter instance
   * 
   * @param shooter structure that used to initialize gimbal, refer to type shooter_t
   */
  Shooter(shooter_t shooter);

  /**
   * @brief set the speed of accelerating motors
   * 
   * @param speed 
   */
  void SetFlywheelSpeed(float speed);

  /**
   * @brief load the next bullet
   * 
   * @return int servomotor status, refer to type servo_status_t
   */
  int LoadNext();

  /**
   * @brief calculate the output of the motors under current configuration
   * 
   */
  void CalcOutput();

private:
  bool fly_using_can_motor_;
  MotorCANBase* left_fly_can_motor_;
  MotorCANBase* right_fly_can_motor_;
  MotorPWMBase* left_fly_pwm_motor_;
  MotorPWMBase* right_fly_pwm_motor_;
  bool left_fly_motor_invert_;
  bool right_fly_motor_invert_;
  ServoMotor* load_servo_;
  float load_step_angle_;

  PIDController left_pid_;
  PIDController right_pid_;

  float left_fly_speed_;
  float right_fly_speed_;
  float load_angle_;
};

}
