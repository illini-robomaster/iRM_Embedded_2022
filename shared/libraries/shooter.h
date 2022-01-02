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

typedef enum{
	MOTOR_Left = 0,
	MOTOR_Rright = 1,
	MOTOR_Bullet = 2,
} shooter_motor_t;

typedef enum{
	HERO = 0,
	STANDARD = 1,
	ENGINEER = 2,
	AERIAL = 3,
	SENTRY = 4,
	DART = 5,
	RADAR = 6
} robot_type_t;

class Shooter {
public:
    Shooter(robot_type_t robot_type, const int* motor_id, const float* fire_pid, const float* load_pid);
    void Fire(float speed);
    void Load(float speed);

private:
    PIDController pid_Left;
    PIDController pid_Right;
    PIDController pid_Bullet;
    MotorCANBase** motors;
    robot_type_t type;
};

}
