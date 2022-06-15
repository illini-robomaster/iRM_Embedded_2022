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
#include "i2c.h"

#include "bsp_print.h"
#include "bsp_imu.h"
#include "bsp_laser.h"
#include "cmsis_os.h"

#include "dbus.h"
#include "protocol.h"

#include "shooter.h"
#include "gimbal.h"
#include "chassis.h"

static const int SHOOTER_TASK_DELAY = 10;
static const int GIMBAL_TASK_DELAY = 1;
static const int CHASSIS_TASK_DELAY = 10;

static const float CHASSIS_DEADZONE = 0.08;

static volatile float ch0;
static volatile float ch1;
static volatile float ch2;
static volatile float ch3;

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;
static bsp::Laser* laser = nullptr;

//=====================================================================================================
// IMU
//=====================================================================================================

#define IMU_RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL); }
};

static IMU* imu = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & IMU_RX_SIGNAL) {  // unnecessary check
      imu->Update();
    }
  }
}

//=====================================================================================================
// Referee
//=====================================================================================================

#define REFEREE_RX_SIGNAL (1 << 1)

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

class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

communication::Referee* referee = nullptr;
RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

//=====================================================================================================
// Shooter
//=====================================================================================================

const osThreadAttr_t shooterTaskAttribute = {.name = "shooterTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t shooterTaskHandle;

static control::MotorCANBase* sl_motor = nullptr;
static control::MotorCANBase* sr_motor = nullptr;
static control::MotorCANBase* ld_motor = nullptr;
static control::Shooter* shooter = nullptr;

void shooterTask(void* arg) {
  UNUSED(arg);

  while (true) {
    osDelay(SHOOTER_TASK_DELAY);
  }
}

//=====================================================================================================
// Gimbal
//=====================================================================================================

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

control::gimbal_data_t* gimbal_param = nullptr;

//static control::MotorCANBase* pitch_motor = nullptr;
//static control::MotorCANBase* yaw_motor = nullptr;
//static control::Gimbal* gimbal = nullptr;

void gimbalTask(void* arg) {
  UNUSED(arg);

//  control::MotorCANBase* gimbal_motors[] = {pitch_motor, yaw_motor};
//  UNUSED(gimbal_motors);

  while (!imu->DataReady());
  osDelay(5000);

  while (true) {
    osDelay(GIMBAL_TASK_DELAY);
  }
}

//=====================================================================================================
// Chassis
//=====================================================================================================

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

static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;

void chassisTask(void* arg) {
  UNUSED(arg);

  while (true) {
    osDelay(CHASSIS_TASK_DELAY);
  }
}

//=====================================================================================================
// RM Init
//=====================================================================================================

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);
  dbus = new remote::DBUS(&huart3);
  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);

  bsp::IST8310_init_t IST8310_init;
  IST8310_init.hi2c = &hi2c3;
  IST8310_init.int_pin = DRDY_IST8310_Pin;
  IST8310_init.rst_group = GPIOG;
  IST8310_init.rst_pin = GPIO_PIN_6;
  bsp::BMI088_init_t BMI088_init;
  BMI088_init.hspi = &hspi1;
  BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
  BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
  BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
  BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;
  bsp::heater_init_t heater_init;
  heater_init.htim = &htim10;
  heater_init.channel = 1;
  heater_init.clock_freq = 1000000;
  heater_init.temp = 45;
  bsp::IMU_typeC_init_t imu_init;
  imu_init.IST8310 = IST8310_init;
  imu_init.BMI088 = BMI088_init;
  imu_init.heater = heater_init;
  imu_init.hspi = &hspi1;
  imu_init.hdma_spi_rx = &hdma_spi1_rx;
  imu_init.hdma_spi_tx = &hdma_spi1_tx;
  imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
  imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;
  imu = new IMU(imu_init, false);

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  sl_motor = new control::Motor3508(can1, 0x202);
  sr_motor = new control::Motor3508(can1, 0x203);
  ld_motor = new control::Motor3508(can1, 0x201);
  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = sl_motor;
  shooter_data.right_flywheel_motor = sr_motor;
  shooter_data.load_motor = ld_motor;
  shooter = new control::Shooter(shooter_data);

//  pitch_motor = new control::Motor6020(can1, 0x205);
//  yaw_motor = new control::Motor6020(can1, 0x206);
//  control::gimbal_t gimbal_data;
//  gimbal_data.pitch_motor = pitch_motor;
//  gimbal_data.yaw_motor = yaw_motor;
//  gimbal_data.model = control::GIMBAL_STANDARD_TWO;
//  gimbal = new control::Gimbal(gimbal_data);
//  gimbal_param = gimbal->GetData();

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
  chassis_data.model = control::CHASSIS_MECANUM;
//  UNUSED(chassis_data);
//  UNUSED(chassis);
  chassis = new control::Chassis(chassis_data);
}

//=====================================================================================================
// RM Thread
//=====================================================================================================

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  shooterTaskHandle = osThreadNew(shooterTask, nullptr, &shooterTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
}

//=====================================================================================================
// RM Default Task
//=====================================================================================================

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  while (true) {
    set_cursor(0, 0);
    clear_screen();

    print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
          imu->DataReady() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
    print("Temp: %.2f\r\n", imu->Temp);
    print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
          imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);

    print("\r\n");

    print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
    print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
    print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);

    print("\r\n");

//    print("Yaw:\r\n");
//    yaw_motor->PrintData();
//    print("Pitch:\r\n");
//    pitch_motor->PrintData();

    osDelay(100);
  }
}

//=====================================================================================================
// END
//=====================================================================================================
