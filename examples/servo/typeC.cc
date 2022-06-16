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

#include "cmsis_os.h"
#include "main.h"
#include "bsp_pwm.h"

#define PWM_CHANNEL 1
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50
#define INIT_PULSE_WIDTH 900

uint32_t pulse_width = INIT_PULSE_WIDTH;

bsp::PWM* motor1;

void RM_RTOS_Init(void) {

  motor1 = new bsp::PWM(&htim1, PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, INIT_PULSE_WIDTH);

}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  motor1->Start();
  while (1) {
    motor1->SetPulseWidth(pulse_width);
    osDelay(500);
    if (pulse_width == INIT_PULSE_WIDTH) {
      osDelay(1000);
    }
    pulse_width += 100;
    if (pulse_width > 2100) {
      pulse_width = INIT_PULSE_WIDTH;
    }
  }
}  
