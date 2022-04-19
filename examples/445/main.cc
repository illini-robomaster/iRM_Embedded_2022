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
#include "main.h"

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "bsp_imu_i2c.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "chassis.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;
control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;
static const uint16_t IMUAddress = 0x50;
static bsp::IMU *imu = nullptr;
static bsp::GPIO *gpio_red, *gpio_green;

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

const osThreadAttr_t readTaskAttribute = {.name = "readTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 128 * 4,
                                          .priority = (osPriority_t)osPriorityNormal,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t readTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(readTaskHandle, RX_SIGNAL); }
};

CustomUART* esp32_uart = nullptr;

void RM_RTOS_Init() {
 print_use_uart(&huart8);

 gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
 gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

 imu = new bsp::IMU(&hi2c2, IMUAddress);

 can1 = new bsp::CAN(&hcan1, 0x201, true);
 can2 = new bsp::CAN(&hcan2, 0x201, false);

 pitch_motor = new control::Motor6020(can1, 0x205);
 yaw_motor = new control::Motor6020(can2, 0x206);

 control::gimbal_t gimbal_data;
 gimbal_data.pitch_motor = pitch_motor;
 gimbal_data.yaw_motor = yaw_motor;
 gimbal_data.model = control::GIMBAL_STANDARD_2022_ALPHA;
 gimbal = new control::Gimbal(gimbal_data);

 fl_motor = new control::Motor3508(can2, 0x201);
 fr_motor = new control::Motor3508(can2, 0x202);
 bl_motor = new control::Motor3508(can2, 0x203);
 br_motor = new control::Motor3508(can2, 0x204);
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

 esp32_uart = new CustomUART(&huart6);
 esp32_uart->SetupRx(300);
 esp32_uart->SetupTx(300);
}

const char cmd0[] = "hld";
const char cmd1[] = "chs";
const char cmd2[] = "gim";
const char cmd3[] = "sho";

void readTask(void* arg) {
  UNUSED(arg);
//  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
//      length = esp32_uart->Read(&data);
      gpio_red->Toggle();
      gpio_green->Toggle();
      esp32_uart->Read(&data);
      if (strncmp((char*)data, cmd0, 3) == 0) {
        print("hold");
      } else if (strncmp((char*)data, cmd1, 3) == 0) {
        print("chassis");
      } else if (strncmp((char*)data, cmd2, 3) == 0) {
        print("gimbal");
      } else if (strncmp((char*)data, cmd3, 3) == 0) {
        print("shooter");
      }
    }
  }
}

void RM_RTOS_Threads_Init(void) {
  readTaskHandle = osThreadNew(readTask, nullptr, &readTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
 UNUSED(args);

 osDelay(500);  // DBUS initialization needs time

 control::MotorCANBase* motors_can1_pitch[] = {pitch_motor};
 control::MotorCANBase* motors_can2_yaw[] = {yaw_motor};
 control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

// control::gimbal_data_t gimbal_data = gimbal->GetData();

 if (!imu->IsRead())
   RM_ASSERT_TRUE(false, "IMU Init Failed!\r\n");

 float angle[3];
 float pitch_target = 0, yaw_target = 0;
 float pitch_curr, yaw_curr;

 while (true) {
   if (dbus->swl == remote::UP || dbus->swl == remote::DOWN)
     RM_ASSERT_TRUE(false, "Operation killed\r\n");

   if (!(imu->GetAngle(angle)))
     RM_ASSERT_TRUE(false, "I2C Error!\r\n");

   if (dbus->swr == remote::UP) {
     float pitch_ratio = dbus->ch3 / 600.0;
     float yaw_ratio = -dbus->ch2 / 600.0;
     pitch_target = wrap<float>(pitch_target + pitch_ratio / 40.0, -PI, PI);
     yaw_target = wrap<float>(yaw_target + yaw_ratio / 30.0, -PI, PI);
   }
   pitch_curr = -angle[1];
   yaw_curr = angle[2];
   float yaw_diff;
   if (-PI < yaw_target && yaw_target < -PI / 2 && PI / 2 < yaw_curr && yaw_curr < PI) {
     yaw_diff = yaw_target - yaw_curr + 2 * PI;
   } else if (-PI < yaw_curr && yaw_curr < -PI / 2 && PI / 2 < yaw_target && yaw_target < PI) {
     yaw_diff = yaw_target - yaw_curr - 2 * PI;
   } else {
     yaw_diff = yaw_target - yaw_curr;
   }
//   if (dbus->swr == remote::UP) {
//     gimbal->TargetAbs(pitch_ratio * gimbal_data.pitch_max_, yaw_ratio * gimbal_data.yaw_max_);
//   } else if (dbus->swr == remote::MID) {
//     gimbal->TargetRel(pitch_ratio / 60, yaw_ratio / 60);
//   }
   float yaw_offset = 0;
   if (dbus->swr == remote::MID) {
     chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
     yaw_offset = dbus->ch2 / 660.0 * 2 * PI / 6;
   }

   gimbal->TargetRel((pitch_target - pitch_curr) / 18, (yaw_diff + yaw_offset) / 30);
   gimbal->Update();
   control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
   control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

   chassis->Update();
   control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

   osDelay(5);
 }
}
