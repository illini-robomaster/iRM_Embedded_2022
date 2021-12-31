//
// Created by neo on 12/28/21.
//

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
