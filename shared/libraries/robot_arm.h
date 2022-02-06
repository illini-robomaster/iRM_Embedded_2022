
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
#define WITH_CONTROLLER
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#ifdef WITH_CONTROLLER
#include "dbus.h"
#endif
#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2
#define SPEED (4 * PI)
#define ACCELERATION (2 * PI)
#define MAX_MOTOR_NUM 3

#include "bsp_laser.h"
#include "bsp_print.h"
#include "cmsis_os.h"

#ifndef IRM_EMBEDDED_2022_ROBOT_ARM_H
#define IRM_EMBEDDED_2022_ROBOT_ARM_H

namespace control {
  typedef enum { ROBOT_ARM_STANDARD_ZERO } robot_arm_model_t;

  typedef struct {
    MotorCANBase** motors; /* motor instances of all robot_arm motors */
    robot_arm_model_t model;

  } robot_arm_t;

  struct Motors {
    enum {left, right, gripper};
  };

  struct Servos {
    enum {left, right, gripper};
  };

class Robot_arm {
  public:
    Robot_arm(const robot_arm_t robot_arm);
    ~Robot_arm();

 private:
    MotorCANBase** motors_;
    robot_arm_model_t model_;
    PIDController pids_[MAX_MOTOR_NUM];
    float* speeds_;
  };
}


#endif  // IRM_EMBEDDED_2022_ROBOT_ARM_H