/*
 * LED_Breath.cpp
 *
 *  Created on: Aug 21, 2021
 *      Author: BillY
 */


#include "LED_PWM.h"

extern TIM_HandleTypeDef htim1;

Module::LED_PWM* LED_Red;

static const uint32_t LED_Red_timer_freq = 168E6;
static const uint32_t LED_Red_output_freq = 656250;

void LED_PWM_Init() {
	LED_Red = new Module::LED_PWM(&htim1, TIM_CHANNEL_2, LED_Red_timer_freq, LED_Red_output_freq, 0);
	LED_Red->Start();
}

