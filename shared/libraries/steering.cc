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

#include <cmath>
#include "steering.h"

namespace control {
  SteeringChassis::SteeringChassis(const steering_chassis_t* _chassis) : chassis(_chassis) {
    radius = _chassis->radius;
    vx = 0.0;
    vy = 0.0;
    vw = 0.0;
    
  }

  SteeringChassis::~SteeringChassis() {
    chassis = nullptr;
  }

  void SteeringChassis::SetXSpeed(double _vx) : vx(_vx) {}

  void SteeringChassis::SetYSpeed(double _vy) : vy(_vy) {}

  void SteeringChassis::SetWSpeed(double _vw) : vw(_vw) {}

  void SteeringChassis::Update() {

    // compute angle and speed
    double v1 = sqrt(pow(vy - vw * cos(PI/4), 2.0) + pow(vx - vw * sin(PI/4), 2.0));
    double v2 = sqrt(pow(vy - vw * cos(PI/4), 2.0) + pow(vx + vw * sin(PI/4), 2.0));
    double v3 = sqrt(pow(vy + vw * cos(PI/4), 2.0) + pow(vx + vw * sin(PI/4), 2.0));
    double v4 = sqrt(pow(vy + vw * cos(PI/4), 2.0) + pow(vx - vw * sin(PI/4), 2.0));
    
    double _theta1 = atan2(vy - vw * cos(PI/4), vx - vw * sin(PI/4));
    double _theta2 = atan2(vy - vw * cos(PI/4), vx + vw * sin(PI/4));
    double _theta3 = atan2(vy + vw * cos(PI/4), vx + vw * sin(PI/4));
    double _theta4 = atan2(vy + vw * cos(PI/4), vx - vw * sin(PI/4));

    // update with relative angles (-180 - 180)
    chassis->fl_steer_motor->TurnRelative(clip<double>(_theta1 - theta1, -PI, PI));
    chassis->fr_steer_motor->TurnRelative(clip<double>(_theta2 - theta2, -PI, PI));
    chassis->bl_steer_motor->TurnRelative(clip<double>(_theta3 - theta3, -PI, PI));
    chassis->br_steer_motor->TurnRelative(clip<double>(_theta4 - theta4, -PI, PI));

    theta1 = _theta1;
    theta2 = _theta2;
    theta3 = _theta3;
    theta4 = _theta4;
    
  }
}

