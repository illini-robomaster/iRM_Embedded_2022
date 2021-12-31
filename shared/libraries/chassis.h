//
// Created by neo on 12/28/21.
//

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
