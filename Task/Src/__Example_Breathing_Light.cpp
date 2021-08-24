/*
 * __Example_Breathing_Light.cpp
 *
 *  Created on: Aug 21, 2021
 *      Author: BillY
 */


#include "TaskConfig.h"

#if INCLUDE_Example_Breathing_Light

#include "__Example_Breathing_Light.h"
#include "LED_PWM.h"
#include "Button.h"

extern Module::LED_PWM* LED_Red;
extern Module::Button* Button;

static bool press = false;

__NO_RETURN void Example_Breathing_Light(void* argument) {
	LED_PWM_Init();
	Button_Init();
	unsigned bright = 0;
	while (true) {
		if (press) {
			for (unsigned i = 0; i < 2; ++i) {
				while(bright < LED_Red->GetPeriod()){
			        LED_Red->SetPulseWidth(++bright);
					osDelay(2);
			    }
			    while(bright > 0){
			        LED_Red->SetPulseWidth(--bright);
				    osDelay(2);
			    }
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

