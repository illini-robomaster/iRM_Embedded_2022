/*
 * LED_Breath.h
 *
 *  Created on: Aug 21, 2021
 *      Author: BillY
 */

#ifndef INC_LED_PWM_H_
#define INC_LED_PWM_H_


#include "PWM.h"

void LED_PWM_Init();

namespace Module {
	class LED_PWM : public BSP::PWM {
	public:
		using BSP::PWM::PWM;
	};
}


#endif /* INC_LED_PWM_H_ */
