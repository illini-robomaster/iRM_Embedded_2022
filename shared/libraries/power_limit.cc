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
  double a = 0;
  double b = 0;
  double c = 0;
  for (int i = 0; i < motor_num_; ++i) {
    a += pow(PID_output[i] / 16384, 2);
    b += abs(PID_output[i] / 16384 * vel_real[i]);
    c += pow(vel_real[i], 2);
  }
  a *= effort_coeff_;
  c = c * velocity_coeff_ - max_power_;
  double zoom_coeff = (pow(b, 2) - 4 * a * c) > 0 ? ((-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a)) : 0;
  for (int i = 0; i < motor_num_; ++i) {
    output[i] = zoom_coeff > 1 ? PID_output[i] : PID_output[i] * zoom_coeff;
  }
}

PowerLimitNaive::PowerLimitNaive(int motor_num, power_limit_t* param) {
  motor_num_ = motor_num;
  power_limit_ = param->power_limit;
  WARNING_power_ = param->WARNING_power;
  WARNING_power_buff_ = param->WARNING_power_buff;
  buffer_total_current_limit_ = param->buffer_total_current_limit;
  power_total_current_limit_ = param->power_total_current_limit;
}

void PowerLimitNaive::Output(float chassis_power, float chassis_power_buffer, float* PID_output, float* output) {
  float total_current_limit;
  if(chassis_power_buffer < WARNING_power_buff_) {
    float power_scale;
    if(chassis_power_buffer > 5.0f) {
      //scale down WARNING_POWER_BUFF
      //缩小WARNING_POWER_BUFF
      power_scale = chassis_power_buffer / WARNING_power_buff_;
    } else {
      //only left 10% of WARNING_POWER_BUFF
      power_scale = 5.0f / WARNING_power_buff_;
    }
    //scale down
    //缩小
    total_current_limit = buffer_total_current_limit_ * power_scale;
  } else {
    //power > WARNING_POWER
    //功率大于WARNING_POWER
    if(chassis_power > WARNING_power_) {
      float power_scale;
      //power < 80w
      //功率小于80w
      if(chassis_power < power_limit_) {
        //scale down
        //缩小
        power_scale = (power_limit_ - chassis_power) / (power_limit_ - WARNING_power_);
      } else {
      //power > 80w
      //功率大于80w
        power_scale = 0.0f;
      }
      total_current_limit = buffer_total_current_limit_ + power_total_current_limit_ * power_scale;
    } else {
    //power < WARNING_POWER
    //功率小于WARNING_POWER
      total_current_limit = buffer_total_current_limit_ + power_total_current_limit_;
    }
  }
  float total_current = 0;
  for (int i = 0; i < motor_num_; ++i) {
    total_current += fabs(PID_output[i]);
  }
  if (total_current > total_current_limit) {
    float current_scale = total_current_limit / total_current;
    for (int i = 0; i < motor_num_; ++i) {
      output[i] = PID_output[i] * current_scale;
    }
  } else {
    for (int i = 0; i < motor_num_; ++i) {
      output[i] = PID_output[i];
    }
  }
}

}
