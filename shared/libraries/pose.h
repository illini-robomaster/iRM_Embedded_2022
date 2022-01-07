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

#include "bsp_imu.h"

namespace control {

class Pose {
public:
  Pose(bsp::MPU6500* _imu);
  void PoseInit(void);
  void GetPose(bsp::vec3f_t *_pose);
  void SetAlpha(float _alpha);
  void ComplementaryFilterUpdate(void);

private:
  bsp::MPU6500* imu;
  // last pose
  float x;
  float y;
  float z;
  // last timestamp
  uint32_t timestamp;
  
  float alpha;
   
}; // class Pose end


} // ns end
