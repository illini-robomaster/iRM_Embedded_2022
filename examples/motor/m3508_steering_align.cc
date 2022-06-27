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
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define TEST_SPEED (0.4 * PI)

bsp::CAN* can = nullptr;
control::MotorCANBase* motor = nullptr;
float transmission_ratio;
control::ConstrainedPID pid;

static bsp::GPIO* input = nullptr;

bool steering_align_detect() {
  // Fill in the detection for photoelectric switch
  return !input->Read();
}

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  input = new bsp::GPIO(IN4_GPIO_Port, IN4_Pin);

  // Fill in corresponding CAN, motor ID, transmission ratio, and omega PID
  can = new bsp::CAN(&hcan1, 0x201, true);
  motor = new control::Motor3508(can, 0x204);

  transmission_ratio = 8;
  float* omega_pid_param = new float[3]{140, 1.2, 25};
  float max_iout = 1000;
  float max_out = 13000;
  pid.Reinit(omega_pid_param, max_iout, max_out);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};

  float start_angle = -1;
  float end_angle = -1;
  bool pos_edge_aligned = false;
  bool neg_edge_aligned = false;
  BoolEdgeDetector edge_detect(true);

  print("Alignment Begin\r\n");
  while (!pos_edge_aligned || !neg_edge_aligned) {
    edge_detect.input(steering_align_detect());
    if (!pos_edge_aligned && edge_detect.posEdge()) {
      start_angle = motor->GetTheta();
      pos_edge_aligned = true;
    } else if (pos_edge_aligned && !neg_edge_aligned && edge_detect.negEdge()) {
      end_angle = motor->GetTheta();
      neg_edge_aligned = true;
    }
    motor->SetOutput(
        pid.ComputeConstrainedOutput(motor->GetOmegaDelta(TEST_SPEED * transmission_ratio)));
    control::MotorCANBase::TransmitOutput(motors, 1);
  }
  motor->SetOutput(0);
  control::MotorCANBase::TransmitOutput(motors, 1);

  float angle_diff = wrap<float>(end_angle - start_angle, -PI, PI);
  float aligned_angle = wrap<float>(start_angle + angle_diff, 0, 2 * PI);
  print("Alignment End, <offset_angle = %10.7f>, %.2f %.2f\r\n", aligned_angle, start_angle,
        end_angle);
}
