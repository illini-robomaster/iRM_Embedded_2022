/*
 * Buzzer.cpp
 *
 *  Created on: 2021年8月22日
 *      Author: BillY
 */


#include "Buzzer.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim12;

Module::Buzzer* Buzzer;

static const uint32_t Buzzer_timer_freq = 1E5;

void Buzzer_Init() {
	Buzzer = new Module::Buzzer(&htim12, TIM_CHANNEL_1, Buzzer_timer_freq);
}

namespace Module {
	Buzzer::Buzzer(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t timer_freq) :
	PWM_(htim, channel, timer_freq, 0, 0) {}

	void Buzzer::SingSong(const BuzzerNoteDelayed* notes) {
		PWM_.Start();
		while (notes->note != BuzzerNote::Finish) {
			SingTone(notes->note);
			osDelay(notes->delay);
			++notes;
		}
		PWM_.Stop();
	}

	void Buzzer::SingTone(const BuzzerNote& note) {
		if (note != BuzzerNote::Finish) {
			PWM_.SetFrequency((uint32_t)note);
			PWM_.SetPulseWidth(PWM_.GetPeriod() / 2);
		}
	}
}

