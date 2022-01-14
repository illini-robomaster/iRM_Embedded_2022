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

/**
 * @brief gimbal models 
 */
typedef enum {
  CHASSIS_STANDARD_ZERO
} chassis_model_t;

typedef struct {
	MotorCANBase** motors;
	float** pid_params; 
	chassis_model_t model;
} chassis_t;

struct FourWheel {
  enum {
		front_left,
		front_right,
		back_left,
		back_right,
		motor_num
	};
};

class Chassis {
 public:
	Chassis(const chassis_t chassis);

	~Chassis();

	void SetSpeed(const float x_speed, const float y_speed, const float turn_speed);

	void Update();

 private:
	// acquired from user
	MotorCANBase** motors_;
	chassis_model_t model_;

	PIDController** pids_;
	float* speeds_;
};

}
