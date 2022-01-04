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
#define LEGACY_GIMBAL_POFF 4.725f /*!< Lagacy gimbal pitch offset */
#define LEGACY_GIMBAL_YOFF 3.406f /*!< Lagacy gimbal yaw offset   */
/**
  * @}
  */

typedef struct {
  MotorCANBase* pitch_motor;
  MotorCANBase* yaw_motor;
  float pitch_offset;
  float yaw_offset;
  float pitch_Kp;
  float pitch_Ki;
  float pitch_Kd;
  float yaw_Kp;
  float yaw_Ki;
  float yaw_Kd;
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

    float pitch_angle_;
    float yaw_angle_;

    PIDController pitch_pid_;
    PIDController yaw_pid_;
};

} // namespace control