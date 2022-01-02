/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2020 RoboMaster.                                          *
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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define TARGET_SPEED1 0
#define TARGET_SPEED2 80

bsp::CAN* can1 = NULL;
control::MotorCANBase* motor = NULL;
BoolEdgeDetecter detecter(false);

void RM_RTOS_Init() {
  print_use_uart(&huart8);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};
  control::PIDController pid(20, 15, 30);

  bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  float target = TARGET_SPEED1;

  while (1) {
    detecter.input(key.Read());
    if (detecter.posEdge()) {
      target = TARGET_SPEED2;
      pid.Reset();
    } else if (detecter.negEdge()) {
      target = TARGET_SPEED1;
      pid.Reset();
    }

    float diff = motor->GetOmegaDelta(target);
    int16_t out = pid.ComputeConstraintedOutput(diff);
    motor->SetOutput(out);
    control::MotorCANBase::TransmitOutput(motors, 1);
    // motor->PrintData();
    print("%10.4f %6d\r\n", diff, out);
    osDelay(10);
  }
}
