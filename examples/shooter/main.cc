//
// Created by neo on 12/30/21.
//

#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "main.h"
#include "dbus.h"
#include "shooter.h"

#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG

remote::DBUS* dbus = nullptr;
control::Shooter* shooter = nullptr;

void RM_RTOS_Init() {
	dbus = new remote::DBUS(&huart1);
	int motor_id[3] = {1, 2, 3};
	float fire_pid[3] = {80, 3, 0.1};
	float load_pid[3] = {30, 5, 0.1};
	shooter= new control::Shooter(control::HERO, motor_id, fire_pid, load_pid);
}

void RM_RTOS_Default_Task(const void* args) {
	UNUSED(args);
	bsp::GPIO laser(LASER_GPIO_Port, LASER_Pin);
	while (true) {
		laser.High();
		shooter->Fire(dbus->ch1);
		osDelay(10);
		shooter->Load(dbus->ch3);
		osDelay(10);
	}
}
