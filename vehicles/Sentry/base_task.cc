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
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "rgb.h"
#include "protocol.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH (2 * PI / 4)
#define SPEED 50
#define ACCELERATION (20 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::ServoMotor* servo = nullptr;
BoolEdgeDetector key_detector(false);
static BoolEdgeDetector FakeDeath(false);
static remote::DBUS* dbus = nullptr;
static display::RGB* RGB = nullptr;
static const uint32_t color_red = 0xFFFF0000;
static const uint32_t color_green = 0xFF00FF00;
static const uint32_t color_blue = 0xFF0000FF;
static volatile bool Dead = false;

bsp::GPIO* gpio_red;


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


const osThreadAttr_t baseTaskAttribute = {.name = "baseTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 512 * 4,
                                          .priority = (osPriority_t)osPriorityNormal,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t baseTaskHandle;

// declare for the base Task
void baseTask(void* argument);

void RM_RTOS_Threads_Init(void) {
  baseTaskHandle = osThreadNew(baseTask, nullptr, &baseTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201);

  referee_uart = new CustomUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);

  referee = new communication::Referee;

  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);
  motor = new control::Motor3508(can1, 0x201);
  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{140, 1.2, 25};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  servo = new control::ServoMotor(servo_data);

  dbus = new remote::DBUS(&huart3); // change to uart 3 for type C; 1 for type A
}

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

// dummy main thread to test multi-tasks
void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  float maxPower = 0;
  uint16_t minBuffer = 200;
//  RGB->Display(color_green);
  while (true) {
    FakeDeath.input(dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
        Dead = true;
        RGB->Display(color_red);
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

    osDelay(100);
  }
}

void baseTask(void* argument) {
  UNUSED(argument);
  osDelay(500);  // DBUS initialization needs time

  while (true) {
    RGB->Display(color_blue);
    if (dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  RGB->Display(color_green);

  // real pos
  float target = 0;
  // want-to-go pos
  float curr_target = 0.0;

  control::MotorCANBase* motors[] = {motor};
//  int totalLength = 2093;
//  int offset = 290;
  float direction = 1.0;
//  const float MAX = (totalLength - 2 * offset) / (100 * PI) * 2;
//  const float MIN = 0;

//  const float MAX = (totalLength - 2 * offset) / (100 * PI) ;
//  const float MIN = - (totalLength - 2 * offset) / (100 * PI);

  const float MAX = 2 * PI;
  const float MIN = - 2 * PI;
  float step = PI;
  while (true) {
    while (Dead) osDelay(100);
    // target = float(dbus->ch1) / remote::DBUS::ROCKER_MAX * 6 * PI;
    curr_target += direction * step;
    // servo->GetTheta(); you can use this to get the absolute pos of the motor
    // if servo command not rejected
    // use FALSE to wait until last command finished
    if (servo->SetTarget(curr_target, false) != 0) {
      target = curr_target;
      if (target >= MAX || target <= MIN) {
        direction = -direction;
      }
    } else {
      curr_target = target;
    }
    servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);
//    print("Max: %.2f     Min: %.2f\r\n", MAX, MIN);
    osDelay(2);
  }
}