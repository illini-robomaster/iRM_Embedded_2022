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
	ENGINEER = 2
} chassis_type_t;

class Chassis {
public:
    Chassis(chassis_type_t robot_type);
    void Move(double x, double y, double z);

private:
    control::PIDController** pid;
    MotorCANBase** motors;
    chassis_type_t type;
};

}
