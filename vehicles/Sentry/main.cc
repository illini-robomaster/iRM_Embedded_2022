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

/* ===== BEGIN RTOS TASK DEF ===== */
const osThreadAttr_t gimbalTaskAttribute = {.name = "GimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 1024 * 4,
                                            .priority = (osPriority_t)osPriorityRealtime1,
                                            .tz_module = 0,
                                            .reserved = 0};

const osThreadAttr_t jetsonSendTaskAttribute = {.name = "JetsonSendTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityRealtime1,
                                            .tz_module = 0,
                                            .reserved = 0};

osThreadId_t gimbalTaskHandle;
osThreadId_t jetsonSendTaskHandle;
/* =====  END RTOS TASK DEF  ===== */

std::unique_ptr<CustomUART> uart;

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  uint32_t length;
  uint8_t* data;

  uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(200);
  uart->SetupTx(200);

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      length = uart->Read(&data);

      miniPCreceiver.Receive(data, length);
      if (miniPCreceiver.GetFlag() == 1) {
        miniPCreceiver.GetPayLoad(buffer);
      }
    }
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
      gimbal->TargetRel(-buffer[1] / 100000.0, buffer[0] / 100000.0);
      buffer[0] = 0;
      buffer[1] = 0;
    }
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(1);
  }
}

void jetsonSendTask(void* arg) {
  // tell Jetson the status of embedded system
  // (e.g., enemy team; IMU state; timestamp)
  // receiving is done in another task for efficiency
  UNUSED(arg);

  osDelay(100); // wait for UART init

  while (true) {
    // 'B' for Blue; 'R' for Red enemy team
    // FIXME: read from referee system!
    uint8_t enemy_team = uint8_t(char('B')); // 'R' or 'B'
    uart->Write(&enemy_team, 1);
    osDelay(1000);
  }
}

void RM_RTOS_Threads_Init(void) {
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  jetsonSendTaskHandle = osThreadNew(jetsonSendTask, nullptr, &jetsonSendTaskAttribute);
}
