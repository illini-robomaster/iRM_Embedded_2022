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
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "minipc.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define SPEED (50 * PI)
#define TEST_SPEED (0.5 * PI)
#define ACCELERATION (100 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::ServoMotor* servo = nullptr;
remote::DBUS* dbus = nullptr;

bsp::GPIO* key = nullptr;
BoolEdgeDetector key_detector(false);
bsp::GPIO* gpio_red;
bsp::GPIO* gpio_green;

auto miniPCreceiver = communication::MiniPCProtocol();
uint32_t buffer[2] = {0};

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  return key && key->Read() == 1;
}

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
  print_use_uart(&huart6);
  bsp::SetHighresClockTimer(&htim2);

  can1 = new bsp::CAN(&hcan1, 0x205);
  motor = new control::Motor3508(can1, 0x205);

  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = 8;
  servo_data.omega_pid_param = new float[3]{140, 1.2, 25};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  servo = new control::ServoMotor(servo_data);

  dbus = new remote::DBUS(&huart1);

  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  uint32_t length;
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart8);  // see cmake for which uart
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
        gpio_red->High();
        miniPCreceiver.GetPayLoad(buffer);
        print("Pitch: %10d Yaw: %10d\r\n", buffer[0], buffer[1]);
          gpio_green->High();
      } else {
          gpio_red->High();
      }
    } else {
        gpio_green->High();
        gpio_red->High();
    }
    osDelay(200);
  }
}

void motorTask(void* arg) {
  UNUSED(arg);
  control::MotorCANBase* motors[] = {motor};
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  osDelay(500);  // DBUS initialization needs time
  
  while (true) {
    servo->SetTarget(buffer[0] / 10e6, true);
    servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}

const osThreadAttr_t motorTaskAttribute = {.name = "UITask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 1024 * 4,
                                           .priority = (osPriority_t)osPriorityBelowNormal,
                                           .tz_module = 0,
                                           .reserved = 0};

osThreadId_t motorTaskHandle;

void RM_RTOS_Threads_Init(void) {
  motorTaskHandle = osThreadNew(motorTask, nullptr, &motorTaskAttribute);
}
