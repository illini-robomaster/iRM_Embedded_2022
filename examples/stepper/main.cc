/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "main.h"

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "stepper.h"

bsp::GPIO *key = nullptr;
control::Stepper* stepper = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);
  stepper = new control::Stepper(&htim1, 1, 1000000, DIR_GPIO_Port, DIR_Pin,ENABLE_GPIO_Port, ENABLE_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);
  //unsigned speed = 2000; // Empty Weight Max 2100
//  int length = 1000;
//  //bool direction = false;
//    while (true) {
//      if (!key->Read()) {
//          stepper->Enable();
//          osDelay(2000);
//        } else {
//          stepper->Disable();
//          osDelay(2000);
//        }
//        osDelay(length);
//        stepper->Stop();
//      osDelay(100);
//    }

  unsigned speed = 500; // Empty Weight Max 2100
  int length = 1400;
  //bool direction = false;

  while (true) {
    stepper->Move(control::BACKWARD, speed);
    osDelay(length);
    osDelay(200);
    stepper->Move(control::FORWARD, speed);
    osDelay(length);
//    if (direction) {
//      stepper->Move(control::FORWARD, speed);
//    } else {
//      stepper->Move(control::BACKWARD, speed);
//    }

    //osDelay(length);
    osDelay(200);
  }


//  while (true && i > 0) {
//    direction = !direction;
//    if (!direction) {
//      //test_dir->High();
//      stepper->Move(control::FORWARD, speed);
//    } else {
//      //test_dir->Low();
//      stepper->Move(control::BACKWARD, speed);
//    }
//    osDelay(length);
//    stepper->Stop();
//    osDelay(100);
//    i--;
//  }
}

/*
 * A4988 步进电机驱动 接线说明
 * 1B Green
 * 1A Black
 * 2A Red
 * 2B Blue
 * STEP PWM
 * MISO MOSI
 * PB14 PB15
 * DIR  ENABLE
 */