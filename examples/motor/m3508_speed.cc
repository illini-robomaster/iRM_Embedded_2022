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

// If want controller to be used
#define WITH_CONTROLLER

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#ifdef WITH_CONTROLLER
#include "dbus.h"
#endif

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#ifdef WITH_CONTROLLER
#define TARGET_SPEED 160
#else
#define TARGET_SPEED1 0
#define TARGET_SPEED2 80
#endif

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
#ifdef WITH_CONTROLLER
remote::DBUS* dbus = nullptr;
#else
BoolEdgeDetector detector(false);
#endif

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);

#ifdef WITH_CONTROLLER
  dbus = new remote::DBUS(&huart1);
#endif
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
#ifdef WITH_CONTROLLER
  osDelay(500);  // DBUS initialization needs time
#endif

  control::MotorCANBase* motors[] = {motor};
  control::PIDController pid(20, 15, 30);

  bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

#ifdef WITH_CONTROLLER
  float target = 0;
#else
  float target = TARGET_SPEED1;
#endif

  while (true) {
#ifdef WITH_CONTROLLER
    if (dbus->ch3 > 10) {
      target = TARGET_SPEED;
    } else if (dbus->ch3 < -10) {
      target = -TARGET_SPEED;
    } else {
      target = 0;
    }
#else
    detector.input(key.Read());
    if (detector.posEdge()) {
      target = TARGET_SPEED2;
      pid.Reset();
    } else if (detector.negEdge()) {
      target = TARGET_SPEED1;
      pid.Reset();
    }
#endif
    float diff = motor->GetOmegaDelta(target);
    int16_t out = pid.ComputeConstraintedOutput(diff);
    motor->SetOutput(out);
    control::MotorCANBase::TransmitOutput(motors, 1);
    motor->PrintData();
    osDelay(10);
  }
}
