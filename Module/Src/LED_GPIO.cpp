/*
 * LED.cpp
 *
 *  Created on: 2021年8月19日
 *      Author: BillY
 */


#include "LED_GPIO.h"

Module::LED_GPIO* LED_Green;
Module::LED_GPIO* LED_Line[8];

void LED_GPIO_Init() {
	LED_Green = new Module::LED_GPIO(LED_Green_GPIO_Port, LED_Green_Pin);
	LED_Line[0] = new Module::LED_GPIO(LED0_GPIO_Port, LED0_Pin);
	LED_Line[1] = new Module::LED_GPIO(LED1_GPIO_Port, LED1_Pin);
	LED_Line[2] = new Module::LED_GPIO(LED2_GPIO_Port, LED2_Pin);
	LED_Line[3] = new Module::LED_GPIO(LED3_GPIO_Port, LED3_Pin);
	LED_Line[4] = new Module::LED_GPIO(LED4_GPIO_Port, LED4_Pin);
	LED_Line[5] = new Module::LED_GPIO(LED5_GPIO_Port, LED5_Pin);
	LED_Line[6] = new Module::LED_GPIO(LED6_GPIO_Port, LED6_Pin);
	LED_Line[7] = new Module::LED_GPIO(LED7_GPIO_Port, LED7_Pin);
}

