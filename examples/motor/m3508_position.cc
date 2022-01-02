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

#define NOTCH               (2 * PI / 2)
#define GEARBOX_MULTIPLIER  19.22
#define SPEED               80

bsp::CAN* can1 = NULL;
control::MotorCANBase* motor = NULL;
BoolEdgeDetecter key_detecter(false);
BoolEdgeDetecter behavior_detecter(false);

void RM_RTOS_Init() {
  print_use_uart(&huart8);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};
  control::PIDController pid_move(20, 15, 30);
  control::PIDController pid_hold(20, 40, 0);

  bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);
  osDelay(500);

  float target = 0;
  float align_angle = motor->GetTheta();
  FloatEdgeDetecter wrap_detecter(align_angle, PI);
  float angle_offset = 0;

  float motor_angle = 0, total_angle = 0;
  bool hold = true;
  while (1) {
    key_detecter.input(key.Read());
    if (key_detecter.posEdge()) {
      target = wrap<float>(target + NOTCH, -PI, PI);
      hold = false;
      print("move\r\n");
      pid_move.Reset();
      pid_hold.Reset();
    }

    motor_angle = wrap<float>(motor->GetTheta() - align_angle, -PI, PI);
    wrap_detecter.input(motor_angle);
    if (wrap_detecter.negEdge()) {
      angle_offset += 2 * PI / GEARBOX_MULTIPLIER;
      angle_offset = wrap<float>(angle_offset, -PI, PI);
    } else if (wrap_detecter.posEdge()) {
      angle_offset -= 2 * PI / GEARBOX_MULTIPLIER;
      angle_offset = wrap<float>(angle_offset, -PI, PI);
    }

    total_angle = angle_offset + motor_angle / GEARBOX_MULTIPLIER;
  
    float diff = wrap<float>(target - total_angle, -PI, PI);
    
    int16_t out;

    if (abs(diff) < 0.05)
      hold = true;

    behavior_detecter.input(hold);
    if (behavior_detecter.posEdge()) {
      print("hold\r\n");
      pid_move.Reset();
    }


    if (hold) {
      out = pid_hold.ComputeConstraintedOutput(diff * GEARBOX_MULTIPLIER);
      out += pid_move.ComputeConstraintedOutput(motor->GetOmegaDelta(0));
      out = clip<float>(out, -32768, 32767);
      motor->SetOutput(out);
    } else {
      out = pid_move.ComputeConstraintedOutput(motor->GetOmegaDelta(SPEED));
      motor->SetOutput(out);
    }
    control::MotorCANBase::TransmitOutput(motors, 1);

    static int i = 0;
    if (i++ > 2) {
      // print("%10.4f %10.4f\r\n", total_angle, angle_offset);
      i = 0;
    }
    // static float last = 0;
    // print("%10.4f\r\n", motor_angle - last);
    // last = motor_angle;

    osDelay(10);
  }
}
