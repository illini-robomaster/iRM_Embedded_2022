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

#include <cstring>
#include <memory>

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "main.h"

#define RX_SIGNAL (1 << 0)

using std::min;

extern osThreadId_t defaultTaskHandle;

static bsp::GPIO *gpio_red, *gpio_green;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  uint32_t length;
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart8);  // see cmake for which uart
  uart->SetupRx(50);
  uart->SetupTx(50);

  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

  // constant length pkg

  // const uint8_t PAYLOAD_SIZE = 8;
  const uint8_t PKG_LEN = 16;
  uint8_t host_command[PKG_LEN];
  int index = -1;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = uart->Read(&data);

      // if read anything, flash red
      gpio_red->High();

      if (index >= 0) {
        // already found header
        int remain = min((int)PKG_LEN - index, (int)length);
        memcpy(host_command + index, data, remain);
        index += remain;

        if (index == PKG_LEN - 1) {
          // done package reading
          index = -1;
          // package handling here!TODO:
          goto handle;
        }
      } else

        for (int32_t i = 0; i < (int32_t)length; i++) {
          if ((data[i] == 'S' && data[i + 1] == 'T') || (data[i] == 'M' && data[i + 1] == 'Y')) {
            index = 0;

            memcpy(host_command, data + i, index = min((int)PKG_LEN, (int)(length - i)));
            if (index == PKG_LEN) {
              // handling here! TODO:
              index = -1;
              goto handle;
              break;
            }
          }
        }

      if (false) {
      handle:

        gpio_green->High();
        uart->Write((uint8_t*)host_command, 16);
      }

      osDelay(200);
      gpio_green->Low();
      gpio_red->Low();
    }
  }
}
