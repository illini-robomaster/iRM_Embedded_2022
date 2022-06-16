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
#include "servo.h"

#define PWM_CHANNEL 1
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50
#define SNAIL_IDLE_THROTTLE 900

control::Servo* servo;

void RM_RTOS_Init(void) {  
  servo = new control::Servo(&htim1, PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  servo->Start();
  while (1) {
    servo->SetAngle(0.0); 
    osDelay(1000);
    servo->SetAngle(30.0);
    osDelay(1000);
    servo->SetAngle(60.0);
    osDelay(1000);
    servo->SetAngle(90.0);
    osDelay(1000);
    servo->SetAngle(120.0);
    osDelay(1000);
  }
  servo->Stop();
}  
