/*
 * GPIT_Button.cpp
 *
 *  Created on: Aug 16, 2021
 *      Author: BillY
 */


#include "TaskConfig.h"

#if INCLUDE_Example_Flowing_Lights

#include "__Example_Flowing_Lights.h"
#include "LED_GPIO.h"
#include "Button.h"

extern Module::LED_GPIO* LED_Line[8];
extern Module::Button* Button;

static bool press = false;

__NO_RETURN void Example_Flowing_Lights(void* argument) {
	LED_GPIO_Init();
	Button_Init();
	while (true) {
		if (press) {
			for (auto & i : LED_Line) {
				i->Toggle();
				osDelay(100);
			}
			press = false;
		}
		osDelay(50);
	}
}

void Module::Button::IntCallback() {
	press = true;
}

#endif

