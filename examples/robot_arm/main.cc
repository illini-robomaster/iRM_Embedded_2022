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

#define WITH_CONTROLLER

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#ifdef WITH_CONTROLLER
#include "dbus.h"
#endif

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH (2 * PI / 4)
#define SPEED (2 * PI)
#define ACCELERATION (1 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motorL = nullptr;
control::MotorCANBase* motorR = nullptr;
control::MotorCANBase* motorG = nullptr;
control::ServoMotor* servoL = nullptr;
control::ServoMotor* servoR = nullptr;
control::ServoMotor* servoG = nullptr;
#ifdef WITH_CONTROLLER
remote::DBUS* dbus = nullptr;
#else
BoolEdgeDetector key_detector(false);
#endif

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

#ifdef WITH_CONTROLLER
 dbus = new remote::DBUS(&huart1);
#endif
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
    set_cursor(0, 0);
    clear_screen();
    print("servoL: ");
    servoL->PrintData();
    print("servoR: ");
    servoR->PrintData();
    print("servoG: ");
    servoG->PrintData();
    osDelay(500);
  }
}
/* init new task END */

void RM_RTOS_Threads_Init(void) {
  servoPrintTaskHandle = osThreadNew(servoPrint_Task, nullptr, &servoPrintTask_attributes);
}


void RM_RTOS_Default_Task(const void* args) {
 UNUSED(args);
#ifdef WITH_CONTROLLER
 osDelay(500);  // DBUS initialization needs time
#endif

 control::MotorCANBase* motors[] = {motorL, motorR, motorG};
 bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);

 float targetLR = 0.0;
 float targetG = 0.0;

 while (true) {
#ifdef WITH_CONTROLLER
   targetG += float(dbus->ch1) / remote::DBUS::ROCKER_MAX * 2 * PI / 100;
   servoG->SetTarget(targetG, true);
   
   targetLR += float(dbus->ch3) / remote::DBUS::ROCKER_MAX * PI / 100;
   if (targetLR >= 0 && targetLR <= 2.3) {
     servoL->SetTarget(targetLR, true);
     servoR->SetTarget(-targetLR, true);
   }
   

#else
   key_detector.input(key.Read());
   if (key_detector.posEdge() && servo->SetTarget(target) != 0) {
     target = wrap<float>(target + NOTCH, 0, 2 * PI);
   }
#endif
   control::MotorCANBase::TransmitOutput(motors, 3);
   
   servoL->CalcOutput();
   servoR->CalcOutput();
   servoG->CalcOutput();   
   
   osDelay(10);
 }
}
