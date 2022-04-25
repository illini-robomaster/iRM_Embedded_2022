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
#include "shooter.h"
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
control::MotorCANBase* sl_motor = nullptr;
control::MotorCANBase* sr_motor = nullptr;
control::MotorCANBase* ld_motor = nullptr;
control::Shooter* shooter = nullptr;
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

  sl_motor = new control::Motor3508(can1, 0x202);
  sr_motor = new control::Motor3508(can1, 0x203);
  ld_motor = new control::Motor3508(can1, 0x201);
  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = sl_motor;
  shooter_data.right_flywheel_motor = sr_motor;
  shooter_data.load_motor = ld_motor;
  shooter = new control::Shooter(shooter_data);

  dbus = new remote::DBUS(&huart1);

  esp32_uart = new CustomUART(&huart6);
  esp32_uart->SetupRx(300);
  esp32_uart->SetupTx(300);
}

const char cmd0[] = "hld";
const char cmd1[] = "chs";
const char cmd2[] = "gim";
const char cmd3[] = "sho";

bool run_gimbal = false;
bool run_chassis = false;
bool run_shooter = false;
int16_t ch0 = 0;
int16_t ch1 = 0;
int16_t ch2 = 0;
int16_t ch3 = 0;

void readTask(void* arg) {
  UNUSED(arg);
//  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      gpio_red->Toggle();
      gpio_green->Toggle();
      int len = esp32_uart->Read(&data);
      print("Len: <%d>, ", len);
      if (len < 4) continue;
      run_gimbal = false; run_chassis = false; run_shooter = false;
      ch0 = 0; ch1 = 0; ch2 = 0; ch3 = 0;
      if (strncmp((char*)data, cmd0, 3) == 0) {
        print("hold:");
        run_gimbal = true;
        run_chassis = true;
      } else if (strncmp((char*)data, cmd1, 3) == 0) {
        ch0 = data[4] << 8 | data[5];
        ch1 = data[6] << 8 | data[7];
        ch2 = data[8] << 8 | data[9];
        run_chassis = true;
        print("chassis: %d, %d, %d", ch0, ch1, ch2);
      } else if (strncmp((char*)data, cmd2, 3) == 0) {
        ch3 = data[4] << 8 | data[5];
        ch2 = data[6] << 8 | data[7];
        run_gimbal = true;
        print("gimbal: %d, %d", ch3, ch2);
      } else if (strncmp((char*)data, cmd3, 3) == 0) {
        run_shooter = true;
        print("shooter:");
      }
      print("\r\n");
      // ch0 = 0;
      // ch1 = 0;
      // ch2 = 0;
      // ch3 = 0;
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
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

// control::gimbal_data_t gimbal_data = gimbal->GetData();

  if (!imu->IsRead())
    RM_ASSERT_TRUE(false, "IMU Init Failed!\r\n");

 float angle[3], angle_offset[3];
 float pitch_target = 0, yaw_target = 0;
 float pitch_curr, yaw_curr;

  print("Calirate\r\n");
  for (int i = 0; i < 1000; i++) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
    osDelay(5);
  }
  if (!(imu->GetAngle(angle_offset)))
    RM_ASSERT_TRUE(false, "I2C Error!\r\n");
  print("Begin\r\n");

  while (true) {
    if (!(imu->GetAngle(angle)))
      RM_ASSERT_TRUE(false, "I2C Error!\r\n");

    angle[0] = wrap<float>(angle[0] - angle_offset[0], -PI, PI);
    angle[1] = wrap<float>(angle[1] - angle_offset[1], -PI, PI);
    angle[2] = wrap<float>(angle[2] - angle_offset[2], -PI, PI);

    if (run_gimbal) {
      float pitch_ratio = ch3 / 600.0;
      float yaw_ratio = -ch2 / 600.0;
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
    
    float yaw_offset = 0;
    if (run_chassis) {
      chassis->SetSpeed(ch0, ch1, ch2);
      yaw_offset = ch2 / 660.0 * 2 * PI / 6;
    }

    if (run_shooter) {
      shooter->SetFlywheelSpeed(600);
      shooter->LoadNext();
    } else {
      shooter->SetFlywheelSpeed(0);
    }

    gimbal->TargetRel((pitch_target - pitch_curr) / 18, (yaw_diff + yaw_offset) / 30);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

    chassis->Update();
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);

    osDelay(5);
  }
}


//  if (!imu->IsRead())
//    RM_ASSERT_TRUE(false, "IMU Init Failed!\r\n");

//  float angle[3];
//  float pitch_target = 0, yaw_target = 0;
//  float pitch_curr, yaw_curr;

//  while (true) {
//    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN)
//      RM_ASSERT_TRUE(false, "Operation killed\r\n");

//    if (!(imu->GetAngle(angle)))
//      RM_ASSERT_TRUE(false, "I2C Error!\r\n");

//    if (dbus->swr == remote::UP) {
//      float pitch_ratio = dbus->ch3 / 600.0;
//      float yaw_ratio = -dbus->ch2 / 600.0;
//      pitch_target = wrap<float>(pitch_target + pitch_ratio / 40.0, -PI, PI);
//      yaw_target = wrap<float>(yaw_target + yaw_ratio / 30.0, -PI, PI);
//    }
//    pitch_curr = -angle[1];
//    yaw_curr = angle[2];
//    float yaw_diff;
//    if (-PI < yaw_target && yaw_target < -PI / 2 && PI / 2 < yaw_curr && yaw_curr < PI) {
//      yaw_diff = yaw_target - yaw_curr + 2 * PI;
//    } else if (-PI < yaw_curr && yaw_curr < -PI / 2 && PI / 2 < yaw_target && yaw_target < PI) {
//      yaw_diff = yaw_target - yaw_curr - 2 * PI;
//    } else {
//      yaw_diff = yaw_target - yaw_curr;
//    }
   
//    float yaw_offset = 0;
//    if (dbus->swr == remote::MID) {
//      chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
//      yaw_offset = dbus->ch2 / 660.0 * 2 * PI / 6;
//    }

//    gimbal->TargetRel((pitch_target - pitch_curr) / 18, (yaw_diff + yaw_offset) / 30);
//    gimbal->Update();
//    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
//    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

//    chassis->Update();
//    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);
   