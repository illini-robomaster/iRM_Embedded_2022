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

#include <memory>

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "oled.h"

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;
static display::OLED *OLED = nullptr;
static volatile float roll;
static volatile float pitch;
static volatile float yaw;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

void refereeTask(void* arg) {
  UNUSED(arg);

  while (true) {
    OLED->DrawIMUData(roll, yaw, pitch);
    OLED->RefreshGram();
    osDelay(100);
  }
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  OLED = new display::OLED(&hi2c2, 0x3C);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  print_use_uart(&huart1);

  uint32_t length;
  uint8_t* data;


  auto uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(50);
  uart->SetupTx(50);

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = uart->Read(&data);
      if (length > 10 && data[0] == 0x55 && data[1] == 0x53) {
        roll = ((data[3]<<8)|data[2])/32768*180;
        pitch= ((data[5]<<8)|data[4])/32768*180;
        yaw  = ((data[7]<<8)|data[6])/32768*180;
      }
    }
    osDelay(50);
  }
}
