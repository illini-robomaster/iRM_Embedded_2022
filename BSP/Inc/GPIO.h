/*
 * gpio.h
 *
 *  Created on: Aug 12, 2021
 *      Author: BillY
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_


#include "main.h"
#include <map>

namespace BSP {
	class GPIO {
	public:
		GPIO(GPIO_TypeDef* group, uint16_t pin);

		bool Read();

		void High();
		void Low();
		void Toggle();

	private:
		GPIO_TypeDef* group_;
		uint16_t pin_;
	};

	class GPIT {
    public:
        explicit GPIT(uint16_t pin);

        virtual void IntCallback() = 0;
        static void IntCallback(uint16_t pin);

	private:
        static std::map<uint16_t, GPIT*> map_;
        uint16_t pin_;
	};
}


#endif /* INC_GPIO_H_ */
