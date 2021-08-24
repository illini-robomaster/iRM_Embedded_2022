/*
 * Button.cpp
 *
 *  Created on: 2021年8月19日
 *      Author: BillY
 */


#include "Button.h"

Module::Button* Button;

void Button_Init() {
	Button = new Module::Button(Button_Pin);
}

