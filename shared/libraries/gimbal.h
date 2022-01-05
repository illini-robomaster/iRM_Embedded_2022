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
#include "can.h"
#include "utils.h"

namespace control {

  
/** @defgroup Transmission Ratios of DJI motors, reference to motor manuals.
* @{
*/
#define LEGACY_GIMBAL_POFF 4.725f   /*!< Lagacy gimbal pitch offset */
#define LEGACY_GIMBAL_YOFF 3.406f   /*!< Lagacy gimbal yaw offset   */
#define LEGACY_GIMBAL_PMAX 0.408f   /*!< Lagacy gimbal pitch max    */
#define LEGACY_GIMBAL_YMAX 1.511f   /*!< Lagacy gimbal yaw max      */
/**
  * @}
  */

typedef struct {
  MotorCANBase* pitch_motor;
  MotorCANBase* yaw_motor;
  float pitch_offset;
  float yaw_offset;
  float pitch_max;
  float yaw_max;
  float pitch_proximity;
  float yaw_proximity;
  float pitch_move_Kp;
  float pitch_move_Ki;
  float pitch_move_Kd;
  float yaw_move_Kp;
  float yaw_move_Ki;
  float yaw_move_Kd;
  float pitch_hold_Kp;
  float pitch_hold_Ki;
  float pitch_hold_Kd;
  float yaw_hold_Kp;
  float yaw_hold_Ki;
  float yaw_hold_Kd;
} gimbal_t;

class Gimbal {
  public:
    /**
     * @brief constructor for Gimbal instance
     */
    Gimbal(gimbal_t gimbal);

    /**
     * @brief update the position of gimbal
     */
    void CalcOutput();

    /**
     * @brief set motors to point to a new orientation
     *
     * @param new_pitch new pitch angled
     * @param new_yaw   new yaw angled
     */
    void TargetAbs(float new_pitch, float new_yaw);

    /**
     * @brief set motors to point to a new orientation
     *
     * @param new_pitch new pitch angled
     * @param new_yaw   new yaw angled
     */
    void TargetRel(float new_pitch, float new_yaw);

  private:
    MotorCANBase* pitch_motor_;
    MotorCANBase* yaw_motor_;
    
    float pitch_offset_;
    float yaw_offset_;
    float pitch_max_;
    float yaw_max_;
    float pitch_proximity_;
    float yaw_proximity_;

    float pitch_move_Kp_;
    float pitch_move_Ki_;
    float pitch_move_Kd_;
    float yaw_move_Kp_;
    float yaw_move_Ki_;
    float yaw_move_Kd_;
    float pitch_hold_Kp_;
    float pitch_hold_Ki_;
    float pitch_hold_Kd_;
    float yaw_hold_Kp_;
    float yaw_hold_Ki_;
    float yaw_hold_Kd_;

    float pitch_angle_;
    float yaw_angle_;

    PIDController pitch_pid_;
    PIDController yaw_pid_;
    BoolEdgeDetector pitch_detector_;
    BoolEdgeDetector yaw_detector_;
};

} // namespace control