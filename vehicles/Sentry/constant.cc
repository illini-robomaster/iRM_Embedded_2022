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

#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "controller.h"
#include "motor.h"
#include "utils.h"
#include "bsp_gpio.h"
#include "dbus.h"
#include "protocol.h"

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
static bsp::GPIO* inputLeft = nullptr;
static bsp::GPIO* inputRight = nullptr;
static remote::DBUS* dbus = nullptr;
static BoolEdgeDetector FakeDeath(false);
static volatile bool Dead = false;
static BoolEdgeDetector leftEdge(false);
static BoolEdgeDetector rightEdge(false);

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

static communication::Referee* referee = nullptr;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL); }
};

static CustomUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 256 * 4,
                                          .priority = (osPriority_t)osPriorityNormal,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t chassisTaskHandle;

void KillAll() {
    RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
    control::MotorCANBase *motor_can1_base[] = {motor};

    while (true) {
        FakeDeath.input(dbus->swl == remote::DOWN);
        if (FakeDeath.posEdge()) {
            Dead = false;
            break;
        }
        motor->SetOutput(0);
        control::MotorCANBase::TransmitOutput(motor_can1_base, 1);
        osDelay(100);
    }
}

void chassisTask(void* arg) {
  UNUSED(arg);
  osDelay(500);  // DBUS initialization needs time

  while (true) {
    if (dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  control::MotorCANBase *motors[] = {motor};

  int output = 1000;
  while (true) {
    while (Dead) osDelay(100);
    motor->SetOutput(output);
    control::MotorCANBase::TransmitOutput(motors, 1);
    leftEdge.input(!inputLeft->Read());
    rightEdge.input(!inputRight->Read());
    if (leftEdge.posEdge()){
      output = -1000;
    }
    if (rightEdge.posEdge()){
      output = 1000;
    }
    osDelay(2);
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  referee_uart = new CustomUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;
  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor3508(can1, 0x201);
  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Threads_Init(void) {
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  inputLeft = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  inputRight = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  float maxPower = 0;
  uint16_t minBuffer = 200;

  while (true) {
    FakeDeath.input(dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
      Dead = true;
      print("killed");
      KillAll();
    }

    if (referee->power_heat_data.chassis_power > maxPower)
        maxPower = referee->power_heat_data.chassis_power;
    if (referee->power_heat_data.chassis_power_buffer < minBuffer
        && referee->power_heat_data.chassis_power_buffer > 0)
        minBuffer = referee->power_heat_data.chassis_power_buffer;

    set_cursor(0, 0);
    clear_screen();
    print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
    print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
    print("Chassis Power: %.2f / %d\r\n", referee->power_heat_data.chassis_power,
        referee->game_robot_status.chassis_power_limit);
    print("Chassis Buffer: %d / 200\r\n", referee->power_heat_data.chassis_power_buffer);
    print("Maximum Power: %.3f\r\n", maxPower);
    print("Lowest Buffer Power: %d\r\n", minBuffer);
//    print("Output: %d\r\n", output);

    osDelay(100);
  }
}