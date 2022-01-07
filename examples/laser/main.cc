//
// Created by neo on 1/6/22.
//

#include "bsp_laser.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"

static bsp::Laser* laser;

void RM_RTOS_Init(void) {
	print_use_uart(&huart8);
	laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
	UNUSED(arguments);

	while (true) {
		set_cursor(0, 0);
		clear_screen();
		laser->On();
		print("laser on\r\n");
		osDelay(1000);
		set_cursor(0, 0);
		clear_screen();
		laser->Off();
		print("laser off\r\n");
		osDelay(1000);
	}
}
