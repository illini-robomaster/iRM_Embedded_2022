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

namespace control {

PowerLimit::PowerLimit(int motor_num, power_limit_t* param) {
  motor_num_ = motor_num;
  power_limit_ = param->power_limit;
  WARNING_power_ = param->WARNING_power;
  WARNING_power_buff_ = param->WARNING_power_buff;
  buffer_total_current_limit_ = param->buffer_total_current_limit;
  power_total_current_limit_ = param->power_total_current_limit;
}

void PowerLimit::Output(float chassis_power, float chassis_power_buffer, float* PID_output,
                        float* output) {
  float total_current_limit;
  if (chassis_power_buffer < WARNING_power_buff_) {
    float power_scale;
    if (chassis_power_buffer > 5.0f) {
      // scale down WARNING_POWER_BUFF
      power_scale = chassis_power_buffer / WARNING_power_buff_;
    } else {
      // only left 10% of WARNING_POWER_BUFF
      power_scale = 5.0f / WARNING_power_buff_;
    }
    // scale down
    total_current_limit = buffer_total_current_limit_ * power_scale;
  } else {
    // power > WARNING_POWER
    if (chassis_power > WARNING_power_) {
      float power_scale;
      // power < 80w
      if (chassis_power < power_limit_) {
        // scale down
        power_scale = (power_limit_ - chassis_power) / (power_limit_ - WARNING_power_);
      } else {
        // power > 80w
        power_scale = 0.0f;
      }
      total_current_limit = buffer_total_current_limit_ + power_total_current_limit_ * power_scale;
    } else {
      // power < WARNING_POWER
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

}  // namespace control
