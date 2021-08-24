/*
 * gpio.cpp
 *
 *  Created on: Aug 12, 2021
 *      Author: BillY
 */


#include "GPIO.h"

namespace BSP {
	GPIO::GPIO(GPIO_TypeDef *group, uint16_t pin) : group_(group), pin_(pin) {}

	void GPIO::High() {
	  HAL_GPIO_WritePin(group_, pin_, GPIO_PIN_SET);
	}

	void GPIO::Low() {
	  HAL_GPIO_WritePin(group_, pin_, GPIO_PIN_RESET);
	}

	void GPIO::Toggle() {
	  HAL_GPIO_TogglePin(group_, pin_);
	}

	bool GPIO::Read() {
	  return HAL_GPIO_ReadPin(group_, pin_) == GPIO_PIN_SET;
	}

	std::map<uint16_t, GPIT*> GPIT::map_;

	GPIT::GPIT(uint16_t pin) : pin_(pin) {
		map_.insert(std::pair<uint16_t, GPIT*>(this->pin_, this));
	}

	void GPIT::IntCallback(uint16_t pin) {
		map_.find(pin)->second->IntCallback();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	BSP::GPIT::IntCallback(GPIO_Pin);
}

