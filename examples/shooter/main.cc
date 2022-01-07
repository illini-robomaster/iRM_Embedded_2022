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
#include "bsp_laser.h"
#include "shooter.h"

static remote::DBUS* dbus = nullptr;
static control::Shooter* shooter = nullptr;
static bsp::Laser* laser = nullptr;

void RM_RTOS_Init() {
	dbus = new remote::DBUS(&huart1);
  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
	int motor_id[3] = {1, 2, 3};
	float fire_pid[3] = {80, 3, 0.1};
	float load_pid[3] = {30, 5, 0.1};
	shooter= new control::Shooter(control::HERO, motor_id, fire_pid, load_pid);
}

void RM_RTOS_Default_Task(const void* args) {
	UNUSED(args);
	while (true) {
		laser->On();
		shooter->Fire(dbus->ch1);
		osDelay(10);
		shooter->Load(dbus->ch3);
		osDelay(10);
	}
}
