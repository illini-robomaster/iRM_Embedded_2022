
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


namespace control {

/**
 * @brief structure used when robot arm is initialized
 */
typedef struct {
  MotorCANBase** motors; /* motor instances of all Arm motors */
} robot_arm_t;

/**
 * @brief Motors on the Arm
 */
struct Motors {
  enum {left, right, gripper};
};

//  struct Servos {
//    enum {left, right, gripper};
//  };

/**
 * @brief wrapper class for RobotArm motors
 */
class RobotArm {
  public:
   /**
    * @brief constructor for RobotArm
    *
    * @param robot_arm structure that used to initialize robot arm, refer to type robot_arm_t
    */
    RobotArm(const robot_arm_t robot_arm);

    /**
     * @brief destructor for robot arm
     */
    ~RobotArm();

    /**
     * @brief set the speed of Left, Right and gripper motors
     *
     * @param LR_speed Left and Right motors speed
     * @param G_speed Gripper motor speed
     */
    void SetSpeed(const float LR_speed, const float G_speed);

    /**
     * @brief calculate the output of the motors under current configuration
     */
    void Update();

 private:
    MotorCANBase** motors_;
    // pids and current speeds for each motor on the robot arm
    PIDController pids_[3];
    float* speeds_;
  };
}
