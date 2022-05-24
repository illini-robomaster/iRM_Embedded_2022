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

#define FORTRESS_MOTOR_NUM 2
#define FORTRESS_RISE_SPEED 50
#define FORTRESS_ROTATE_SPEED 400

namespace control {

class Fortress {
 public:
  Fortress(MotorCANBase** motors, MotorCANBase* yaw_motor);
  ~Fortress();
  void Up();
  void Down();
  void StopRise();
  void Rotate();
  void StopRotate();

 private:
  MotorCANBase** motors_;
  PIDController pids_[FORTRESS_MOTOR_NUM];
  MotorCANBase* yaw_motor_;
  PIDController yaw_pid_;
};

}
