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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "motor.h"

#define TARGET_SPEED 40

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

control::MotorCANBase* fl_steer_motor = nullptr;
control::MotorCANBase* fr_steer_motor = nullptr;
control::MotorCANBase* bl_steer_motor = nullptr;
control::MotorCANBase* br_steer_motor = nullptr;

control::MotorCANBase* fl_wheel_motor = nullptr;
control::MotorCANBase* fr_wheel_motor = nullptr;
control::MotorCANBase* bl_wheel_motor = nullptr;
control::MotorCANBase* br_wheel_motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x205, false);

  fl_steer_motor = new control::Motor3508(can1, 0x204);
  fr_steer_motor = new control::Motor3508(can1, 0x203);
  bl_steer_motor = new control::Motor3508(can1, 0x201);
  br_steer_motor = new control::Motor3508(can1, 0x202);

  fl_wheel_motor = new control::Motor3508(can2, 0x208);
  fr_wheel_motor = new control::Motor3508(can2, 0x207);
  bl_wheel_motor = new control::Motor3508(can2, 0x205);
  br_wheel_motor = new control::Motor3508(can2, 0x206);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors_fl_steer[] = {fl_steer_motor};
  control::MotorCANBase* motors_fr_steer[] = {fr_steer_motor};
  control::MotorCANBase* motors_bl_steer[] = {bl_steer_motor};
  control::MotorCANBase* motors_br_steer[] = {br_steer_motor};

  control::MotorCANBase* motors_fl_wheel[] = {fl_wheel_motor};
  control::MotorCANBase* motors_fr_wheel[] = {fr_wheel_motor};
  control::MotorCANBase* motors_bl_wheel[] = {bl_wheel_motor};
  control::MotorCANBase* motors_br_wheel[] = {br_wheel_motor};

  control::MotorCANBase** motors[] = {motors_fl_steer, motors_fr_steer, motors_bl_steer,
                                      motors_br_steer, motors_fl_wheel, motors_fr_wheel,
                                      motors_bl_wheel, motors_br_wheel};

  control::PIDController pid_fl_steer(20, 15, 30);
  control::PIDController pid_fr_steer(20, 15, 30);
  control::PIDController pid_bl_steer(20, 15, 30);
  control::PIDController pid_br_steer(20, 15, 30);

  control::PIDController pid_fl_wheel(20, 15, 30);
  control::PIDController pid_fr_wheel(20, 15, 30);
  control::PIDController pid_bl_wheel(20, 15, 30);
  control::PIDController pid_br_wheel(20, 15, 30);

  control::PIDController pids[] = {pid_fl_steer, pid_fr_steer, pid_bl_steer, pid_br_steer,
                                   pid_fl_wheel, pid_fr_wheel, pid_bl_wheel, pid_br_wheel};

  for (int i = 0; i < 8; ++i) {
    int j = 0;
    while (true) {
      if (++j >= 200) break;
      float diff = motors[i][0]->GetOmegaDelta(TARGET_SPEED);
      int16_t out = pids[i].ComputeConstrainedOutput(diff);
      motors[i][0]->SetOutput(out);
      control::MotorCANBase::TransmitOutput(motors[i], 1);
      osDelay(10);
    }
  }

  while (true) osDelay(100);
}
