# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2022                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

#include "cmsis_os.h"
#include "main.h"
#include "dbus.h"
#include "chassis.h"

remote::DBUS* dbus = nullptr;
control::Chassis* chassis = nullptr;

void RM_RTOS_Init() {
	dbus = new remote::DBUS(&huart1);
	int motor_id[4] = {2, 3, 1, 4};
	float chassis_pid[3] = {20, 8, 2};
	chassis = new control::Chassis(control::STANDARD, motor_id, chassis_pid); // 5 3 0.1 / 20 8 2
}

void RM_RTOS_Default_Task(const void* args) {
	UNUSED(args);
	while (true) {
		chassis->Move(dbus->ch0, dbus->ch1, dbus->ch2);
		osDelay(10);
	}
}
