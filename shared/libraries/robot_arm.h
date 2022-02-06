
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
  typedef struct {
    MotorCANBase** motors; /* motor instances of all robot_arm motors */
  } robot_arm_t;

  struct Motors {
    enum {left, right, gripper};
  };

//  struct Servos {
//    enum {left, right, gripper};
//  };

class RobotArm {
  public:
    RobotArm(const robot_arm_t robot_arm);
    ~RobotArm();

 private:
    MotorCANBase** motors_;
    robot_arm_model_t model_;
    PIDController pids_[MAX_MOTOR_NUM];
    float* speeds_;
  };
}
