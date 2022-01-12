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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "referee.h"

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

const osThreadAttr_t refereeTaskAttribute = {
        .name = "refereeTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t refereeTaskHandle;

class CustomUART : public bsp::UART {
public:
    using bsp::UART::UART;

protected:
    /* notify application when rx data is pending read */
    void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL); }
};

RoboMaster::Referee* referee = nullptr;
CustomUART* referee_uart = nullptr;

void refereeTask (void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
//      length = referee_uart->Read(&data);
//      set_cursor(0, 0);
//      clear_screen();
//      for (uint32_t i = 0; i < length; ++i) {
//        print("%02X ", data[i]);
//      }
//      print("\r\n");
      length = referee_uart->Read(&data);
      referee->Update(data, length);
    }
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);

  referee_uart = new CustomUART(&huart7);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);

  referee = new RoboMaster::Referee;
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("Chassis Volt: %hu, Chassis Curr: %hu", referee->power_heat_data.chassis_volt, referee->power_heat_data.chassis_current);
    osDelay(100);
  }
}
