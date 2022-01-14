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
  float prev_target = wrap<float>(servo->GetTarget() - step_angles_[servo], 0, 2 * PI);
  servo->SetTarget(prev_target, static_cast<servo_mode_t>(-data.dir), true);
}

Shooter::Shooter(shooter_t shooter) {
	left_flywheel_motor_ = shooter.left_flywheel_motor;
	right_flywheel_motor_ = shooter.right_flywheel_motor;
	model_ = shooter.model;

	servo_t servo_data;
  servo_data.motor = shooter.load_motor;

	switch (shooter.model) {
		case SHOOTER_STANDARD_ZERO:
			servo_data.mode = control::SERVO_ANTICLOCKWISE;
			servo_data.speed = 2 * PI;
			servo_data.transmission_ratio = M2006P36_RATIO;
			servo_data.move_Kp = 20;
			servo_data.move_Ki = 15;
			servo_data.move_Kd = 30;
			servo_data.hold_Kp = 40;
			servo_data.hold_Ki = 15;
			servo_data.hold_Kd = 5;

			load_step_angle_ = 2 * PI / 8;
			break;

		case SHOOTER_STANDARD_2022:
			servo_data.mode = control::SERVO_ANTICLOCKWISE;
			servo_data.speed = 2 * PI;
			servo_data.transmission_ratio = M2006P36_RATIO;
			servo_data.move_Kp = 20;
			servo_data.move_Ki = 15;
			servo_data.move_Kd = 30;
			servo_data.hold_Kp = 40;
			servo_data.hold_Ki = 15;
			servo_data.hold_Kd = 5;

			left_pid_ = new PIDController(80, 3, 0.1);
			right_pid_ = new PIDController(80, 3, 0.1);
			flywheel_turning_detector_ = new BoolEdgeDetector(false);
			load_step_angle_ = 2 * PI / 8;
			speed_ = 0;
			break;

		default:
			RM_ASSERT_TRUE(false, "No shooter type specified");
	}
	// Initialize servomotor instance using data provided and register default jam callback
	load_servo_ = new control::ServoMotor(servo_data);
	load_servo_->RegisterJamCallback(jam_callback, 0.6);
	
	// Register in step_angles_ so callback function can find step angle corresponding to
	// specific servomotor instance.  
	step_angles_[load_servo_] = load_step_angle_;
}

Shooter::~Shooter() {
	delete load_servo_;
	load_servo_ = nullptr;

	switch (model_) {
		case SHOOTER_STANDARD_ZERO:
		break;
		case SHOOTER_STANDARD_2022:
		delete left_pid_;
		left_pid_ = nullptr;
		delete right_pid_;
		right_pid_ = nullptr;
		delete flywheel_turning_detector_;
		flywheel_turning_detector_ = nullptr;
	}
}

void Shooter::SetFlywheelSpeed(float speed) {
	switch (model_) {
		case SHOOTER_STANDARD_ZERO:
			left_flywheel_motor_->SetOutput(speed);
			right_flywheel_motor_->SetOutput(speed);
			break;

		case SHOOTER_STANDARD_2022:
			speed_ = speed;
			break;
	}
}

int Shooter::LoadNext() {
	return load_servo_->SetTarget(load_servo_->GetTarget() + load_step_angle_);
}

void Shooter::Update() {
	switch (model_) {
		case SHOOTER_STANDARD_ZERO:
			load_servo_->CalcOutput();
			break;

		case SHOOTER_STANDARD_2022:
			flywheel_turning_detector_->input(speed_ == 0);
			float left_diff = static_cast<MotorCANBase*>(left_flywheel_motor_)->GetOmegaDelta(speed_);
			float right_diff = static_cast<MotorCANBase*>(right_flywheel_motor_)->GetOmegaDelta(-speed_);
			left_flywheel_motor_->SetOutput(left_pid_->ComputeConstraintedOutput(left_diff));
			right_flywheel_motor_->SetOutput(right_pid_->ComputeConstraintedOutput(right_diff));
			load_servo_->CalcOutput();
			break;
	}
}

} // namespace control
