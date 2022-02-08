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

#include "controller.h"
#include "motor.h"

#define MAX_WHEEL_NUM 6

namespace control {

/**
 * @brief chassis models
 */
typedef enum { CHASSIS_STANDARD_ZERO } chassis_model_t;

/**
 * @brief structure used when chassis instance is initialized
 */
typedef struct {
  MotorCANBase** motors; /* motor instances of all chassis motors */
  chassis_model_t model; /* chassis model                         */
} chassis_t;

/**
 * @brief motor configs for four wheel vehicles
 */
struct FourWheel {
  enum { front_left, front_right, back_left, back_right, motor_num };
};

/**
 * @brief wrapper class for chassis
 */
class Chassis {
 public:
  /**
   * @brief constructor for chassis
   *
   * @param chassis structure that used to initialize chassis, refer to type chassis_t
   */
  Chassis(const chassis_t chassis);

  /**
   * @brief destructor for chassis
   */
  ~Chassis();

  /**
   * @brief set the speed for chassis motors
   *
   * @param x_speed chassis speed on x-direction
   * @param y_speed chassis speed on y-direction
   * @param turn_speed chassis clockwise turning speed
   */
  void SetSpeed(const float x_speed, const float y_speed, const float turn_speed);

  /**
   * @brief calculate the output of the motors under current configuration
   * @note does not command the motor immediately
   */
  void Update();

 private:
  // acquired from user
  MotorCANBase** motors_;
  chassis_model_t model_;

  // pids and current speeds for each motor on the chassis
  PIDController pids_[MAX_WHEEL_NUM];
  float* speeds_;
};

}  // namespace control
