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

#include "servo.h"
#include "cmsis_os.h"
#include "bsp_pwm.h"
#include "utils.h"

namespace control {

  Servo::Servo(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq, uint32_t pulse_width) : pulse_width(pulse_width) {
    servo = new bsp::PWM(htim, channel, clock_freq, output_freq, pulse_width);
  }

  Servo::~Servo() {
    delete servo;
    servo = nullptr;
  }
  void Servo::Start() {
    servo->Start();
  }

  void Servo::Stop() {
    servo->Stop();
  }

  void Servo::SetAngle(double _angle) {
    _angle = clip<double>(_angle, MIN_ANGLE, MAX_ANGLE);
    uint32_t p_width = static_cast<uint32_t>(_angle * ANGLE_TO_WIDTH);
    p_width += BASE_WIDTH;
    servo->SetPulseWidth(p_width);
  }

}
