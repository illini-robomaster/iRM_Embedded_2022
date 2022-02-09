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

#include "controller.h"
#include "motor.h"

#define ARM_MOTOR_NUM 3

namespace control {

/**
* @brief structure used when chassis instance is initialized
*/
typedef struct {
 MotorCANBase* motor_left; /* motor instances of all chassis motors */
 MotorCANBase* motor_right;
 MotorCANBase* motor_gripper;
} robotic_arm_t;

/**
* @brief motor configs for four wheel vehicles
*/

enum ArmMotor { left, right, gripper };

/**
* @brief wrapper class for chassis
*/
class RoboticArm {
public:
 /**
  * @brief constructor for chassis
  *
  * @param chassis structure that used to initialize chassis, refer to type chassis_t
  */
 RoboticArm(robotic_arm_t robotic_arm);

 /**
  * @brief destructor for chassis
  */
 ~RoboticArm();

 /**
  * @brief set the speed for chassis motors
  *
  * @param x_speed chassis speed on x-direction
  * @param y_speed chassis speed on y-direction
  * @param turn_speed chassis clockwise turning speed
  */
 void SetPosition(const float arm_position, const float gripper_position);

 /**
  * @brief calculate the output of the motors under current configuration
  * @note does not command the motor immediately
  */
 void Update();

private:
 // acquired from user
 ServoMotor* servo_[ARM_MOTOR_NUM];

 // pids and current speeds for each motor on the chassis
// PIDController pids_[ARM_MOTOR_NUM];
// float* speeds_;
};

}  // namespace control
