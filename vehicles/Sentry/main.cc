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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "bsp_os.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "minipc.h"
#include "gimbal.h"
#include "dbus.h"

#define SPEED (50 * PI)
#define TEST_SPEED (0.5 * PI)
#define ACCELERATION (100 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
remote::DBUS* dbus = nullptr;

auto miniPCreceiver = communication::MiniPCProtocol();
int32_t buffer[2] = {0};

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

void RM_RTOS_Init() {
  bsp::SetHighresClockTimer(&htim5);
  dbus = new remote::DBUS(&huart3);

  can1 = new bsp::CAN(&hcan1, 0x205);
  pitch_motor = new control::Motor3508(can1, 0x205);
  yaw_motor = new control::Motor3508(can1, 0x206);

  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_SENTRY;
  gimbal = new control::Gimbal(gimbal_data);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  uint32_t length;
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(200);
  uart->SetupTx(200);

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = uart->Read(&data);

      miniPCreceiver.Receive(data, length);
      if (miniPCreceiver.GetFlag() == 1) {
        miniPCreceiver.GetPayLoad(buffer);
      }
    }
    osDelay(10);
  }
}

void gimbalTask(void* arg) {
  UNUSED(arg);
  control::MotorCANBase* motors[] = {pitch_motor, yaw_motor};

  osDelay(500);  // initialization needs time

  gimbal->TargetAbs(0, 0);
  gimbal->Update();
  
  while (true) {
    gimbal->TargetRel(dbus->ch1 / 660.0 / 20, dbus->ch0 / 660.0 / 20);
    if (miniPCreceiver.gimbal_moving) {
      // print("Moving: %d Pitch: %10.2f Yaw: %10.2f ", 
      //       miniPCreceiver.gimbal_moving, buffer[0] / 100000.0, buffer[1] / 100000.0);
        
      // print("\r\n");
      gimbal->TargetRel(-buffer[1] / 100000.0, buffer[0] / 100000.0);
      buffer[0] = 0;
      buffer[1] = 0;
    }
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(1);
  }
}

const osThreadAttr_t gimbalTaskAttribute = {.name = "GimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 1024 * 4,
                                            .priority = (osPriority_t)osPriorityBelowNormal,
                                            .tz_module = 0,
                                            .reserved = 0};

osThreadId_t gimbalTaskHandle;

void RM_RTOS_Threads_Init(void) {
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}
