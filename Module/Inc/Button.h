/*
 * Button.h
 *
 *  Created on: 2021年8月19日
 *      Author: BillY
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_


#include "GPIO.h"

void Button_Init();

namespace Module {
	class Button : public BSP::GPIT {
	public:
		using BSP::GPIT::GPIT;
		void IntCallback();
	};
}


#endif /* INC_BUTTON_H_ */
