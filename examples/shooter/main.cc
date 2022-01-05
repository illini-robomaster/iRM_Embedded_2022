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
#include "main.h"
#include "dbus.h"
#include "shooter.h"

#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG

bsp::CAN* can = nullptr;
control::MotorCANBase* left_acc_motor = nullptr;
control::MotorCANBase* right_acc_motor = nullptr;
control::MotorCANBase* load_motor = nullptr;

remote::DBUS* dbus = nullptr;
control::ServoMotor* load_servo = nullptr;
control::Shooter* shooter = nullptr;

void RM_RTOS_Init() {
	dbus = new remote::DBUS(&huart1);

	can = new bsp::CAN(&hcan1, 0x201);
	left_acc_motor = new control::Motor3508(can, 0x201);
	right_acc_motor = new control::Motor3508(can, 0x202);
	load_motor = new control::Motor3508(can, 0x203);

	control::servo_t servo_data;
  servo_data.motor = load_motor;
  servo_data.mode = control::SERVO_ANTICLOCKWISE;
  servo_data.speed = 1.5 * PI;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.move_Kp = 20;
  servo_data.move_Ki = 15;
  servo_data.move_Kd = 30;
  servo_data.hold_Kp = 40;
  servo_data.hold_Ki = 15;
  servo_data.hold_Kd = 5;
	load_servo = new control::ServoMotor(servo_data);

	control::shooter_t shooter_data;
	shooter_data.acc_using_can_motor = true;
	shooter_data.left_acc_can_motor = left_acc_motor;
	shooter_data.right_acc_can_motor = right_acc_motor;
	shooter_data.left_acc_motor_invert = false;
	shooter_data.right_acc_motor_invert = true;
	shooter_data.load_servo = load_servo;
	shooter_data.acc_Kp = 80;
	shooter_data.acc_Ki = 3;
	shooter_data.acc_Kd = 0.1;
	shooter_data.load_step_angle = PI / 8;
	shooter = new control::Shooter(shooter_data);
}

void RM_RTOS_Default_Task(const void* args) {
	UNUSED(args);
  control::MotorCANBase* motors[] = {left_acc_motor, right_acc_motor, load_motor};
	bsp::GPIO laser(LASER_GPIO_Port, LASER_Pin);
	laser.High();

	while (true) {
		shooter->SetAccSpeed(dbus->ch1);
		if (dbus->ch3 > 500)
			shooter->LoadNext();
		shooter->CalcOutput();
		control::MotorCANBase::TransmitOutput(motors, 3);
		osDelay(10);
	}
}
