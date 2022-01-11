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
#if defined(SHOOTER_STANDARD_ZERO)
  MotorPWMBase* left_flywheel_motor;	 /* PWM motor instance of left flywheel motor   */
  MotorPWMBase* right_flywheel_motor; /* PWM motor instance of right flywheel motor  */
#else
  MotorCANBase* left_flywheel_motor;	 /* CAN motor instance of left flywheel motor   */
  MotorCANBase* right_flywheel_motor; /* CAN motor instance of right flywheel motor  */
#endif
  MotorCANBase* load_motor;            /* servomotor instance of load motor           */
  jam_callback_t jam_callback;
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
   * @brief update the output of the motors under current configuration
   * 
   */
  void Update();

private:
  // refer to shooter_t for details
#if defined(SHOOTER_STANDARD_ZERO)
  MotorPWMBase* left_flywheel_motor_;
  MotorPWMBase* right_flywheel_motor_;
#else
  MotorCANBase* left_flywheel_motor_;
  MotorCANBase* right_flywheel_motor_;
  PIDController* left_pid_;  /* pid for left flywheel  */
  PIDController* right_pid_; /* pid for right flywheel */
  BoolEdgeDetector* flywheel_turning_detector_;
  float speed_;
#endif
  ServoMotor* load_servo_;
  float load_step_angle_;
};

}
