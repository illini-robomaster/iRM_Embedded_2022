//
// Created by neo on 12/28/21.
//

#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define TARGET_SPEED 800

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motors[4];

control::Chassis* chassis = nullptr;

void RM_RTOS_Init() {
	can1 = new bsp::CAN(&hcan1, 0x201);
	chassis = new control::Chassis(control::STANDARD);
}


void RM_RTOS_Default_Task(const void* args) {
	UNUSED(args);
	bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);
//	float target;
	while (true) {
//		if (key.Read())
//			target = TARGET_SPEED;
//		else
//			target = 0;
		chassis->Move(TARGET_SPEED, 0, 0);
		osDelay(10);
	}
}