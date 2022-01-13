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
#include "utils.h"
#ifdef WITH_CONTROLLER
#include "dbus.h"
#endif


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH               (2 * PI / 4)
#define SPEED               (2 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::ServoMotor* servo = nullptr;
#ifdef WITH_CONTROLLER
remote::DBUS* dbus = nullptr;
BoolEdgeDetector joystick_detector_abv(false);
BoolEdgeDetector joystick_detector_rgt(false);
BoolEdgeDetector joystick_detector_btm(false);
BoolEdgeDetector joystick_detector_lft(false);
#else
BoolEdgeDetector key_detector(false);
#endif

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);

  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.mode = control::SERVO_NEAREST;
  servo_data.speed = SPEED;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.move_Kp = 30;
  servo_data.move_Ki = 10;
  servo_data.move_Kd = 40;
  servo_data.hold_Kp = 30;
  servo_data.hold_Ki = 25;
  servo_data.hold_Kd = 20;
  servo = new control::ServoMotor(servo_data);

#ifdef WITH_CONTROLLER
  dbus = new remote::DBUS(&huart1);
#endif
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
#ifdef WITH_CONTROLLER
	osDelay(500); // DBUS initialization needs time
#endif

  control::MotorCANBase* motors[] = {motor};
  bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);

  float target = NOTCH;

  while (true) {
    print("%d %d", dbus->ch3, dbus->ch2);
#ifdef WITH_CONTROLLER
    // joystick input range from -660 to 660
    joystick_detector_abv.input(dbus->ch3 > 500);
    joystick_detector_rgt.input(dbus->ch2 < -500);
    joystick_detector_btm.input(dbus->ch3 < -500);
    joystick_detector_lft.input(dbus->ch2 > 500);
    if (joystick_detector_abv.posEdge()) {
      target = 0;
      servo->SetTarget(target);
    } else if (joystick_detector_rgt.posEdge()) {
      target = PI / 2;
      servo->SetTarget(target);
    } else if (joystick_detector_btm.posEdge()) {
      target = PI;
      servo->SetTarget(target);
    } else if (joystick_detector_lft.posEdge()) {
      target = -PI / 2;
      servo->SetTarget(target);
    }
#else
    key_detector.input(key.Read());
    if (key_detector.posEdge() && servo->SetTarget(target) != 0) {
      target = wrap<float>(target + NOTCH, -PI, PI);
    }
#endif
    servo->CalcOutput();
    servo->PrintData();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(10);
  }
}
