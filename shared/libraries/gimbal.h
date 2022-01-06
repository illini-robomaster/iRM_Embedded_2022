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

  
/**
 * @brief offset and max angles of different gimbals
 * @note these should be obtained by reading encoder values through uart/gdb
 */
#define LEGACY_GIMBAL_POFF 4.725f   /* legacy gimbal pitch offset */
#define LEGACY_GIMBAL_YOFF 3.406f   /* legacy gimbal yaw offset   */
#define LEGACY_GIMBAL_PMAX 0.408f   /* legacy gimbal pitch max    */
#define LEGACY_GIMBAL_YMAX 1.511f   /* legacy gimbal yaw max      */

/**
 * @brief structure used when gimbal instance is initialized
 */
typedef struct {
  MotorCANBase* pitch_motor; /* pitch motor instance                                    */
  MotorCANBase* yaw_motor;   /* yaw motor instance                                      */
  float pitch_offset;        /* pitch motor offset                                      */
  float yaw_offset;          /* yaw motor offset                                        */
  float pitch_max;           /* pitch motor max turning angle                           */ 
  float yaw_max;             /* yaw motor max turning angle                             */
  float pitch_proximity;     /* critical pitch diff angle for pid control to toggle     */
  float yaw_proximity;       /* critical yaw diff angle for pid control to toggle       */
  float pitch_move_Kp;       /* Kp of pid that used to control pitch motor when moving  */
  float pitch_move_Ki;       /* Ki of pid that used to control pitch motor when moving  */
  float pitch_move_Kd;       /* Kd of pid that used to control pitch motor when moving  */
  float yaw_move_Kp;         /* Kp of pid that used to control yaw motor when moving    */
  float yaw_move_Ki;         /* Ki of pid that used to control yaw motor when moving    */
  float yaw_move_Kd;         /* Kd of pid that used to control yaw motor when moving    */
  float pitch_hold_Kp;       /* Kp of pid that used to control pitch motor when holding */
  float pitch_hold_Ki;       /* Kd of pid that used to control pitch motor when holding */
  float pitch_hold_Kd;       /* Ki of pid that used to control pitch motor when holding */
  float yaw_hold_Kp;         /* Kp of pid that used to control yaw motor when holding   */
  float yaw_hold_Ki;         /* Ki of pid that used to control yaw motor when holding   */
  float yaw_hold_Kd;         /* Kd of pid that used to control yaw motor when holding   */
} gimbal_t;

/**
 * @brief wrapper class for gimbal
 */
class Gimbal {
  public:
    /**
     * @brief constructor for Gimbal instance
     * 
     * @param gimbal structure that used to initialize gimbal, refer to type gimbal_t
     */
    Gimbal(gimbal_t gimbal);

    /**
     * @brief calculate the output of the motors under current configuration
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
