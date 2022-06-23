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

#include "main.h"

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "motor.h"
#include "fortress.h"

static bsp::GPIO* left = nullptr;
static bsp::GPIO* right = nullptr;

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;

static control::MotorCANBase* motor_left = nullptr;
static control::MotorCANBase* motor_right = nullptr;
static control::MotorCANBase* motor_fortress = nullptr;

static control::Fortress* fortress = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  left = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  right = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x205, false);

  motor_left = new control::Motor3508(can2, 0x205);
  motor_right = new control::Motor3508(can2, 0x208);

  motor_fortress = new control::Motor6020(can2, 0x207);

  control::fortress_t fortress_data;
  fortress_data.leftSwitch = left;
  fortress_data.rightSwitch = right;
  fortress_data.leftElevatorMotor = motor_left;
  fortress_data.rightElevatorMotor = motor_right;
  fortress_data.fortressMotor = motor_fortress;
  fortress = new control::Fortress(fortress_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {motor_left, motor_right};

  while (!fortress->Calibrate()) {
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(2);
  }

  while (true) {
    fortress->Transform(true);
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(2);
  }
}
