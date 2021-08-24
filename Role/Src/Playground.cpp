/*
 * Playground.cpp
 *
 *  Created on: Aug 13, 2021
 *      Author: BillY
 */


#include "RoleConfig.h"

#if INCLUDE_Playground

#include "Playground.h"
#include "cmsis_os.h"

#include "TaskConfig.h"

#if INCLUDE_Example_Flowing_Lights
#include "__Example_Flowing_Lights.h"
static const osThreadAttr_t Example_Flowing_Lights_attributes = {
  .name = "__Example_Flowing_Lights",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
#endif

#if INCLUDE_Example_Breathing_Light
#include "__Example_Breathing_Light.h"
static const osThreadAttr_t Example_Breathing_Light_attributes = {
  .name = "__Example_Breathing_Light",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
#endif

#if INCLUDE_Example_Music
#include "__Example_Music.h"
static const osThreadAttr_t Example_Music_attributes = {
  .name = "__Example_Music",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
#endif

void RoleTask(void* argument) {
	osKernelLock();
#if INCLUDE_Example_Flowing_Lights
	osThreadNew(Example_Flowing_Lights, nullptr, &Example_Flowing_Lights_attributes);
#endif
#if INCLUDE_Example_Breathing_Light
	osThreadNew(Example_Breathing_Light, nullptr, &Example_Breathing_Light_attributes);
#endif
#if INCLUDE_Example_Music
	osThreadNew(Example_Music, nullptr, &Example_Music_attributes);
#endif
	osKernelUnlock();
	osThreadExit();
}

#endif

