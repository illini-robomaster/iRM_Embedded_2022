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
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "minipc.h"
#include "bsp_uart.h"


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define SPEED (10 * PI)
#define TEST_SPEED (0.5 * PI)
#define ACCELERATION (100 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::SteeringMotor* steering = nullptr;
remote::DBUS* dbus = nullptr;

auto miniPCreceiver = communication::MiniPCProtocol();
uint32_t buffer[2] = {0};

bsp::GPIO* key = nullptr;

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  return key->Read() == 1;
}

#define RX_SIGNAL (1 << 0)

static osThreadId_t miniPCTaskHandle;

const osThreadAttr_t miniPCTask_attributes = {.name = "miniPC_Task",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 128 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
void miniPCTask(void* argument);

void RM_RTOS_Threads_Init(void) {
  miniPCTaskHandle = osThreadNew(miniPCTask, nullptr, &miniPCTask_attributes);
}



void RM_RTOS_Init() {
  bsp::SetHighresClockTimer(&htim2);

  can1 = new bsp::CAN(&hcan1, 0x205);
  motor = new control::Motor3508(can1, 0x205);

  control::steering_t steering_data;
  steering_data.motor = motor;
  steering_data.max_speed = SPEED;
  steering_data.test_speed = TEST_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = 8;
  steering_data.offset_angle = 5.96;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.align_detect_func = steering_align_detect;
  steering = new control::SteeringMotor(steering_data);

  dbus = new remote::DBUS(&huart1);

}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  osDelay(500);  // DBUS initialization needs time

  print("Alignment Begin\r\n");
  while (!steering->AlignUpdate()) {
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
  print("\r\nAlignment End\r\n");

  while (true) {
    steering->TurnRelative(static_cast<float>(buffer[0]) / 100000);
    steering->Update();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}



class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(miniPCTaskHandle, RX_SIGNAL); }
};


auto uart = std::make_unique<CustomUART>(&huart8);

void miniPCTask(void* argument) {
  UNUSED(argument);

  uint32_t length;
  uint8_t* data;

  uart->SetupRx(50);
  uart->SetupTx(50);


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
      osDelay(200);
    }
  }
}
