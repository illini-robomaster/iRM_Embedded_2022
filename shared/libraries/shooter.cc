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
#include <unordered_map>

namespace control {

static auto step_angles_ = std::unordered_map<ServoMotor*, float>();

void jam_callback(ServoMotor* servo, const servo_jam_t data) {
	RM_ASSERT_TRUE(step_angles_.find(servo) != step_angles_.end(), "Servo not found");
  float prev_target = wrap<float>(servo->GetTarget() - step_angles_[servo], 0, 2 * PI);
  servo->SetTarget(prev_target, static_cast<servo_mode_t>(-data.dir), true);
}

Shooter::Shooter(shooter_t shooter):
		left_pid_(PIDController(shooter.fly_Kp, shooter.fly_Ki, shooter.fly_Kd)),
		right_pid_(PIDController(shooter.fly_Kp, shooter.fly_Ki, shooter.fly_Kd)) {
	fly_using_can_motor_ = shooter.fly_using_can_motor;
	// Because legacy gimbal uses snail M2305 motors that can only be driven using PWM,
	// interfaces are reserved since the old gimbal could be used for demonstratio 
	// purposes in the future.
	if (shooter.fly_using_can_motor) {
		left_fly_can_motor_ = shooter.left_fly_can_motor;
		right_fly_can_motor_ = shooter.right_fly_can_motor;
		left_fly_motor_invert_ = shooter.left_fly_motor_invert;
		right_fly_motor_invert_ = shooter.right_fly_motor_invert;
	} else {
		left_fly_pwm_motor_ = shooter.left_fly_pwm_motor;
		right_fly_pwm_motor_ = shooter.right_fly_pwm_motor;
	}
	load_servo_ = shooter.load_servo;
	load_step_angle_ = shooter.load_step_angle;
	// Register in step_angles_ so callback function can find step angle corresponding to
	// specific servomotor instance.  
	step_angles_[shooter.load_servo] = shooter.load_step_angle;

	left_fly_speed_ = 0;
	right_fly_speed_ = 0;
	load_angle_ = 0;

	load_servo_->RegisterJamCallback(jam_callback, 50, 0.6);
}

void Shooter::SetFlywheelSpeed(float speed) {
	if (fly_using_can_motor_) {
		left_fly_speed_ = left_fly_motor_invert_ ? -speed : speed;
		right_fly_speed_ = right_fly_motor_invert_ ? -speed : speed;
	} else {
		left_fly_pwm_motor_->SetOutput(speed);
		right_fly_pwm_motor_->SetOutput(speed);
	}
}

int Shooter::LoadNext() {
	int val = load_servo_->SetTarget(load_angle_ + load_step_angle_);
	if (val != 0) {
		load_angle_ = wrap<float>(load_angle_ + load_step_angle_, 0, 2 * PI);
	}
	return val;
}

void Shooter::CalcOutput() {
	if (fly_using_can_motor_) {
		float left_diff = left_fly_can_motor_->GetOmegaDelta(left_fly_speed_);
		float right_diff = right_fly_can_motor_->GetOmegaDelta(right_fly_speed_);
		left_fly_can_motor_->SetOutput(left_pid_.ComputeOutput(left_diff));
		right_fly_can_motor_->SetOutput(right_pid_.ComputeOutput(right_diff));	
	}
	load_servo_->CalcOutput();
}

} // namespace control
