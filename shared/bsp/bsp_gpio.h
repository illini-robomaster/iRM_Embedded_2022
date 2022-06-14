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

#pragma once

/**
 * @note List of GPIOs that can be used on board now
 *    K1: Input,   K2: Output
 *    L1: Input,   L2: Output
 *    M1: Input,   M2: Output
 *    P1: Input,   P2: Output
 *    Q1: Input,   Q2: Output
 */

#include "bsp_error_handler.h"
#include "main.h"

#define NUM_GPITS 16

namespace bsp {

class GPIO {
 public:
  /**
   * @brief constructor for generic GPIO (non interrupt)
   *
   * @param group GPIO group
   * @param pin   GPIO pin number
   */
  GPIO(GPIO_TypeDef* group, uint16_t pin);

  /**
   * @brief Set high output
   */
  void High();

  /**
   * @brief Set low output
   */
  void Low();

  /**
   * @brief Toggle GPIO output
   */
  void Toggle();

  /**
   * @brief Read GPIO input
   *
   * @return 1 for high and 0 for low
   */
  template <bool IsInput = false>
  uint8_t Read();

 private:
  GPIO_TypeDef* group_;
  uint16_t pin_;
  uint8_t state_;
};

class GPIT {
  typedef void (*CallbackTypeDef)(void*);

 public:
  /**
   * @brief Contructor for general purpose interrupt pins
   *
   * @param pin interrupt pin number (x in EXTIx)
   * @param callback callback function pointer to call when interrupt happens
   * @param data associated data for the callback functions
   */
  GPIT(uint16_t pin, CallbackTypeDef callback, void* data);

  void Start();

  void Stop();

 private:
  friend void GPITCallbackWrapper(uint16_t pin);

  uint16_t pin_;
  CallbackTypeDef callback_;
  void* data_;
  bool activated_ = false;

  static int GetGPIOIndex(uint16_t pin);
  static GPIT* gpits[NUM_GPITS];
};

} /* namespace bsp */
