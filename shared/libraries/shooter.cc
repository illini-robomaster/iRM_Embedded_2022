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

#include "shooter.h"
#include "can.h"
#include "bsp_error_handler.h"

namespace control {
	Shooter::Shooter(shooter_t shooter):
			left_pid_(PIDController(shooter.fire_Kp, shooter.fire_Ki, shooter.fire_Kd)),
			right_pid_(PIDController(shooter.fire_Kp, shooter.fire_Ki, shooter.fire_Kd)) {
		fire_using_can_motor_ = shooter.fire_using_can_motor;
		if (shooter.fire_using_can_motor) {
			left_fire_can_motor_ = shooter.left_fire_can_motor;
			right_fire_can_motor_ = shooter.right_fire_can_motor;
			left_fire_motor_invert_ = shooter.left_fire_motor_invert;
			right_fire_motor_invert_ = shooter.right_fire_motor_invert;
		} else {
			left_fire_pwm_motor_ = shooter.left_fire_pwm_motor;
			right_fire_pwm_motor_ = shooter.right_fire_pwm_motor;
		}
		load_servo_ = shooter.load_servo;
		load_step_angle_ = shooter.load_step_angle;

		left_fire_speed_ = 0;
		right_fire_speed_ = 0;
		load_angle_ = 0;
	}

	void Shooter::SetFireSpeed(float speed) {
		if (fire_using_can_motor_) {
			left_fire_speed_ = left_fire_motor_invert_ ? -speed : speed;
			right_fire_speed_ = right_fire_motor_invert_ ? -speed : speed;
		} else {
			left_fire_pwm_motor_->SetOutput(speed);
			right_fire_pwm_motor_->SetOutput(speed);
		}
	}

	int Shooter::LoadNext() {
		int val = load_servo_->SetTarget(load_angle_ + load_step_angle_);
		if (val != 0) {
			load_angle_ = wrap<float>(load_angle_ + load_step_angle_, -PI, PI);
		}
		return val;
	}

	void Shooter::CalcOutput() {
		if (fire_using_can_motor_) {
			float left_diff = left_fire_can_motor_->GetOmegaDelta(left_fire_speed_);
			float right_diff = right_fire_can_motor_->GetOmegaDelta(right_fire_speed_);
			left_fire_can_motor_->SetOutput(left_pid_.ComputeOutput(left_diff));
			right_fire_can_motor_->SetOutput(right_pid_.ComputeOutput(right_diff));	
		}
		load_servo_->CalcOutput();
	}
}
