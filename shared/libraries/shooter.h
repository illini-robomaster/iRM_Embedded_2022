//
// Created by neo on 12/30/21.
//

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
