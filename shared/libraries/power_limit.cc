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

#include "power_limit.h"
#include <cmath>

namespace control{

PowerLimit::PowerLimit(int motor_num, float max_power, float effort_coeff, float velocity_coeff) {
  motor_num_ = motor_num;
  max_power_ = max_power;
  effort_coeff_ = effort_coeff;
  velocity_coeff_ = velocity_coeff;
}

float square(float x) {
  return x * x;
}

void PowerLimit::Output(float* vel_real, float* PID_output, float* output) {
  float a = 0;
  float b = 0;
  float c = 0;
  for (int i = 0; i < motor_num_; ++i) {
    a += square(PID_output[i]);
    b += abs(PID_output[i] * vel_real[i]);
    c += square(vel_real[i]);
  }
  a *= effort_coeff_;
  c = c * velocity_coeff_ - max_power_;
  float zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0;
  for (int i = 0; i < motor_num_; ++i) {
    output[i] = zoom_coeff > 1 ? PID_output[i] : PID_output[i] * zoom_coeff;
  }
}

}
