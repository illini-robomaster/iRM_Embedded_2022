/*
 * LED.h
 *
 *  Created on: 2021年8月19日
 *      Author: BillY
 */

#ifndef INC_LED_GPIO_H_
#define INC_LED_GPIO_H_


#include "GPIO.h"

void LED_GPIO_Init();

namespace Module {
	class LED_GPIO : public BSP::GPIO {
	public:
		using BSP::GPIO::GPIO;
	};
}


#endif /* INC_LED_GPIO_H_ */
