/*
 * PWM.cpp
 *
 *  Created on: 2021年8月21日
 *      Author: BillY
 */


#include "PWM.h"

namespace BSP {
	PWM::PWM() = default;
	PWM::PWM(TIM_HandleTypeDef* htim,
			 uint32_t channel,
			 uint32_t timer_freq,
			 uint32_t output_freq,
			 uint32_t pulse_width) :
			 htim_(htim),
			 channel_(channel),
			 timer_freq_(timer_freq) {
		SetFrequency(output_freq);
		SetPulseWidth(pulse_width);
	}

	void PWM::Start() {
		HAL_TIM_PWM_Start(this->htim_, this->channel_);
	}

	void PWM::Stop() {
		HAL_TIM_PWM_Stop(this->htim_, this->channel_);
	}

	uint32_t PWM::GetPeriod() {
		return __HAL_TIM_GET_AUTORELOAD(this->htim_);
	}

	void PWM::SetFrequency(uint32_t output_freq) {
	  __HAL_TIM_SET_AUTORELOAD(this->htim_, this->timer_freq_ / output_freq - 1);
	  __HAL_TIM_SET_COUNTER(this->htim_, 0);
	}

	void PWM::SetPulseWidth(uint32_t pulse_width) {
	  __HAL_TIM_SET_COMPARE(this->htim_, this->channel_, pulse_width);
	}
}

