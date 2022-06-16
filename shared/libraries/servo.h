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

#include "cmsis_os.h"
#include "bsp_pwm.h"

constexpr double MIN_ANGLE = 0.0;
constexpr double MAX_ANGLE = 120.0;
constexpr uint32_t BASE_WIDTH = 600;
// control pulse width = 600 - 2100.
constexpr double ANGLE_TO_WIDTH = 1.0 / 120 * 1500;
constexpr uint32_t TIM_CLOCK_FREQ = 1000000;
constexpr uint32_t MOTOR_OUT_FREQ = 50;
constexpr uint32_t INIT_PULSE_WIDTH = 600;

namespace control {

  class Servo {
  public: 
    Servo(TIM_HandleTypeDef* htim, uint8_t channel);
    Servo(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq, uint32_t pulse_width);
    ~Servo();
    void Start();
    void Stop();
    void SetAngle(double _angle);

  private:
    bsp::PWM* servo;
    uint32_t pulse_width;
  };

}
