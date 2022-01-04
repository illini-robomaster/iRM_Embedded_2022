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

typedef struct {
	bool fire_using_can_motor;
	MotorCANBase* left_fire_can_motor;
	MotorCANBase* right_fire_can_motor;
	MotorPWMBase* left_fire_pwm_motor;
	MotorPWMBase* right_fire_pwm_motor;
	bool left_fire_motor_invert;
	bool right_fire_motor_invert;
	ServoMotor* load_servo;
	float fire_Kp;
	float fire_Ki;
	float fire_Kd;
	float load_step_angle;
} shooter_t;


class Shooter {
public:
	Shooter(shooter_t shooter);
	void SetFireSpeed(float speed);
	int LoadNext();
	void CalcOutput();

private:
	bool fire_using_can_motor_;
	MotorCANBase* left_fire_can_motor_;
	MotorCANBase* right_fire_can_motor_;
	MotorPWMBase* left_fire_pwm_motor_;
	MotorPWMBase* right_fire_pwm_motor_;
	bool left_fire_motor_invert_;
	bool right_fire_motor_invert_;
	ServoMotor* load_servo_;
	float load_step_angle_;

	PIDController left_pid_;
	PIDController right_pid_;

	float left_fire_speed_;
	float right_fire_speed_;
	float load_angle_;
};

}
