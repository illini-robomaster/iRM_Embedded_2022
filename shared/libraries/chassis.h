# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2022                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

#pragma once

#include "motor.h"
#include "controller.h"

namespace control {

typedef enum{
	MOTOR_FL = 0,
	MOTOR_BL = 1,
	MOTOR_FR = 2,
	MOTOR_BR = 3
} chassis_motor_t;

typedef enum{
	HERO = 0,
	STANDARD = 1,
	ENGINEER = 2,
	AERIAL = 3,
	SENTRY = 4,
	DART = 5,
	RADAR = 6
} robot_type_t;

class Chassis {
public:
    Chassis(robot_type_t robot_type, const int* motor_id, const float* pid);
    void Move(float x, float y, float z);

private:
    PIDController pid_FL;
    PIDController pid_BL;
    PIDController pid_FR;
    PIDController pid_BR;
    MotorCANBase** motors;
    robot_type_t type;
};

}
