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

#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2
BoolEdgeDetector key_detect(false);

#define TARGET_SPEED 80

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;

control::PIDController pid1;
control::OutputConstraintedPIDController pid2;

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);

  float* pid_param = new float[3]{20, 15, 30};
  pid1.Reinit(pid_param);
  pid2.Reinit(pid_param);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  control::MotorCANBase* motors[] = {motor};
  bool use_constrainted_pid = false;

  while (true) {
    float err = motor->GetOmegaDelta(TARGET_SPEED);
    int out1 = pid1.ComputeConstraintedOutput(err);
    int out2 = pid2.ComputeOutput(err, 32767);

    key_detect.input(key.Read());
    if (key_detect.posEdge()) {
      use_constrainted_pid = !use_constrainted_pid;
    }

    if (use_constrainted_pid)
      motor->SetOutput(out2);
    else
      motor->SetOutput(out1);
    control::MotorCANBase::TransmitOutput(motors, 1);

    print("% 10d % 10d % 10d\r\n", out1, out2, out1 - out2);
    osDelay(10);
  }
}
