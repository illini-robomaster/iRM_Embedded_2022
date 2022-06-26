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

PowerLimit::PowerLimit(int motor_num) { motor_num_ = motor_num; }

void PowerLimit::Output(bool turn_on, power_limit_t power_limit_info, float chassis_power,
                        float chassis_power_buffer, float* PID_output, float* output) {
  if (!turn_on) {
    for (int i = 0; i < motor_num_; ++i) output[i] = PID_output[i];
    return;
  }
  float total_current_limit;
  if (chassis_power_buffer < power_limit_info.WARNING_power_buff) {
    float power_scale;
    if (chassis_power_buffer > 5.0f) {
      // scale down WARNING_POWER_BUFF
      power_scale = chassis_power_buffer / power_limit_info.WARNING_power_buff;
    } else {
      // only left 10% of WARNING_POWER_BUFF
      power_scale = 5.0f / power_limit_info.WARNING_power_buff;
    }
    // scale down
    total_current_limit = power_limit_info.buffer_total_current_limit * power_scale;
  } else {
    // power > WARNING_POWER
    if (chassis_power > power_limit_info.WARNING_power) {
      float power_scale;
      // power < 80w
      if (chassis_power < power_limit_info.power_limit) {
        // scale down
        power_scale = (power_limit_info.power_limit - chassis_power) /
                      (power_limit_info.power_limit - power_limit_info.WARNING_power);
      } else {
        // power > 80w
        power_scale = 0.0f;
      }
      total_current_limit = power_limit_info.buffer_total_current_limit +
                            power_limit_info.power_total_current_limit * power_scale;
    } else {
      // power < WARNING_POWER
      total_current_limit =
          power_limit_info.buffer_total_current_limit + power_limit_info.power_total_current_limit;
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
