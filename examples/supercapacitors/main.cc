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

#include "bsp_uart.h"
#include "bsp_print.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "chassis.h"
#include "supercapacitors.h"
#include "protocol.h"

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

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

const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t chassisTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL); }
};

// referee initialization
communication::Referee* referee = nullptr;
CustomUART* referee_uart = nullptr;

//chassis initialization
bsp::CAN* can = nullptr;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

// supercapacitors initialization
power::supercapacitors* supercapacitors = nullptr;


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

void chassisTask(void* arg) {
  UNUSED(arg);
  osDelay(500);  // DBUS initialization needs time
  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  while (true) {
    chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    chassis->Update();
    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(10);
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);

  // referee
  referee_uart = new CustomUART(&huart7);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);

  referee = new communication::Referee;

  // chassis
  can = new bsp::CAN(&hcan2, 0x201, false);
  fl_motor = new control::Motor3508(can, 0x201);
  fr_motor = new control::Motor3508(can, 0x202);
  bl_motor = new control::Motor3508(can, 0x203);
  br_motor = new control::Motor3508(can, 0x204);

  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;

  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_STANDARD_ZERO;
  chassis = new control::Chassis(chassis_data);

  dbus = new remote::DBUS(&huart1);

  // supercapacitor
  supercapacitors = new power::supercapacitors(can);
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    supercapacitors->UpdatePowerLimit(45.0);
    set_cursor(0, 0);
    clear_screen();
    print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
    print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
    print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
//    print("\r\n");
//    print("Shooter Cooling Heat: %hu\r\n", referee->power_heat_data.shooter_id1_17mm_cooling_heat);
//    print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
//    print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
    osDelay(100);
  }
}
