/*
 * PWM.h
 *
 *  Created on: 2021年8月21日
 *      Author: BillY
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_


#include "main.h"

namespace BSP {
	class PWM {
	public:
		PWM();
		PWM(TIM_HandleTypeDef* htim,
			uint32_t channel,
			uint32_t timer_freq,
			uint32_t output_freq,
			uint32_t pulse_width);

		void Start();
		void Stop();

		uint32_t GetPeriod();

		void SetFrequency(uint32_t output_freq);
		void SetPulseWidth(uint32_t pulse_width);

	private:
		TIM_HandleTypeDef* htim_{};
        uint32_t channel_{};
        uint32_t timer_freq_{};
	};
}


#endif /* INC_PWM_H_ */
