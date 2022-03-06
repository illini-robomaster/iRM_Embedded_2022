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
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

#define NOTCH (2 * PI / 4)
#define SPEED (2 * PI)
#define ACCELERATION (1 * PI)

#define gripper_port K2_GPIO_Port
#define gripper_pin K2_Pin

#define RFID_port L2_GPIO_Port
#define RFID_pin L2_Pin

static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* motorL = nullptr;
static control::MotorCANBase* motorR = nullptr;
static control::MotorCANBase* motorG = nullptr;
static control::ServoMotor* servoL = nullptr;
static control::ServoMotor* servoR = nullptr;
static control::ServoMotor* servoG = nullptr;
static bsp::GPIO *gripper, *RFID;
static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;

static remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can1 = new bsp::CAN(&hcan1, 0x201);
  motorL = new control::Motor3508(can1, 0x205);
  motorR = new control::Motor3508(can1, 0x206);
  motorG = new control::Motor3508(can1, 0x207);

  control::servo_t servoL_data;
  servoL_data.motor = motorL;
  servoL_data.mode = control::SERVO_NEAREST;
  servoL_data.max_speed = SPEED;
  servoL_data.max_acceleration = ACCELERATION;
  servoL_data.transmission_ratio = M3508P19_RATIO;
  servoL_data.omega_pid_param = new float[3]{25, 5, 35};
  servoL = new control::ServoMotor(servoL_data);

  control::servo_t servoR_data;
  servoR_data.motor = motorR;
  servoR_data.mode = control::SERVO_NEAREST;
  servoR_data.max_speed = SPEED;
  servoR_data.max_acceleration = ACCELERATION;
  servoR_data.transmission_ratio = M3508P19_RATIO;
  servoR_data.omega_pid_param = new float[3]{25, 5, 35};
  servoR = new control::ServoMotor(servoR_data);

  control::servo_t servoG_data;
  servoG_data.motor = motorG;
  servoG_data.mode = control::SERVO_NEAREST;
  servoG_data.max_speed = SPEED;
  servoG_data.max_acceleration = ACCELERATION;
  servoG_data.transmission_ratio = M3508P19_RATIO;
  servoG_data.omega_pid_param = new float[3]{25, 5, 35};
  servoG = new control::ServoMotor(servoG_data);

  gripper = new bsp::GPIO(gripper_port, gripper_pin);
  RFID = new bsp::GPIO(RFID_port, RFID_pin);

  fl_motor = new control::Motor3508(can1, 0x201);
  fr_motor = new control::Motor3508(can1, 0x202);
  bl_motor = new control::Motor3508(can1, 0x203);
  br_motor = new control::Motor3508(can1, 0x204);

  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;

  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_STANDARD_ZERO;
  chassis = new control::Chassis(chassis_data);

  dbus = new remote::DBUS(&huart1);
}


/* init new task START */
static osThreadId_t servoPrintTaskHandle;

const osThreadAttr_t servoPrintTask_attributes = {.name = "servoPrintTask",
                                                  .attr_bits = osThreadDetached,
                                                  .cb_mem = nullptr,
                                                  .cb_size = 0,
                                                  .stack_mem = nullptr,
                                                  .stack_size = 128 * 4,
                                                  .priority = (osPriority_t)osPriorityNormal,
                                                  .tz_module = 0,
                                                  .reserved = 0};

void servoPrint_Task(void* argument) {
  UNUSED(argument);
  while (true) {
    //    set_cursor(0, 0);
    //    clear_screen();
    //    print("servoL: ");
    //    servoL->PrintData();
    //    print("servoR: "); b
    //    servoR->PrintData();
    //    print("servoG: ");
    //    servoG->PrintData();
    osDelay(500);
  }
}
/* init new task END */

void RM_RTOS_Threads_Init(void) {
  servoPrintTaskHandle = osThreadNew(servoPrint_Task, nullptr, &servoPrintTask_attributes);
}


void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors_arm[] = {motorL, motorR, motorG};
  control::MotorCANBase* motors_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

  float targetLR = 0.0;
  float targetG = 0.0;

//  servoG->SetTarget(0, true);
//  servoL->SetTarget(0, true);
//  servoR->SetTarget(0, true);

  while (true) {
    if (dbus->swl == remote::MID) {
      gripper->Low();
      RFID->Low();
    }
    if (dbus->swl == remote::UP) {
      gripper->High();
    }
    if (dbus->swl == remote::DOWN) {
      RFID->High();
    }

    if (dbus->swr == remote::MID) {
      chassis->SetSpeed(0, 0, 0);
    }
    if (dbus->swr == remote::UP) {
      targetG += float(dbus->ch1) / remote::DBUS::ROCKER_MAX * 2 * PI / 100;
//      servoG->SetTarget(targetG, true);

      targetLR += float(dbus->ch3) / remote::DBUS::ROCKER_MAX * PI / 100;
//      if (targetLR >= 0 && targetLR <= 2.3) {
//        servoL->SetTarget(targetLR, true);
//        servoR->SetTarget(-targetLR, true);
//      }
    }
    if (dbus->swr == remote::DOWN) {
      chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
    }
    chassis->Update();
    servoG->SetTarget(targetG, true);
    servoL->SetTarget(targetLR, true);
    servoR->SetTarget(-targetLR, true);
    servoL->CalcOutput();
    servoR->CalcOutput();
    servoG->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors_arm, 3);
    control::MotorCANBase::TransmitOutput(motors_chassis, 4);
    osDelay(10);
  }
}
