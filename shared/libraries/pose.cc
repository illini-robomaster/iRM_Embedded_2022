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
 
#include "bsp_imu.h"
#include "pose.h"

namespace control {

Pose::Pose(bsp::MPU6500* _imu) {
  if (_imu == nullptr) {
    RM_ASSERT_TRUE(false, "invalid imu");
  }
  // reset private variables
  PoseInit();
  // Set weight for gyro data in complementary filter to 0.98
  SetAlpha(0.98);
}


/** description: reset all accumulated variables to 0
 *  
**/
void Pose::PoseInit(void) {
  timestamp = 0;
  pose.x = 0;
  pose.y = 0;
  pose.z = 0;
}


/** description: read estimated pose
 *  
**/
bsp::vec3f_t Pose::GetPose(void) {
  return pose;
}


/** description: set weight for gyroData
 *  
**/
void Pose::SetAlpha(float _alpha) : alpha(_alpha) {}


/** description: update pose with complementary filter 
 *  note: delta t = (imu->timestamp - timestamp) should be as small as possible
 *        ~<10ms
 *  reference: https://www.pieter-jan.com/node/11
**/
bsp::vec3f_t ComplementaryFilterUpdate(void) {

  pose.x = alpha * (pose.x + imu->gyro.x * (imu->timestamp - timestamp)) + 
           (1 - alpha) * imu->acc.x;
  pose.y = alpha * (pose.y + imu->gyro.y * (imu->timestamp - timestamp)) + 
           (1 - alpha) * imu->acc.y;
  pose.z = alpha * (pose.z + imu->gyro.z * (imu->timestamp - timestamp)) + 
           (1 - alpha) * imu->acc.z;
  
}

} // ns control
