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

#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_imu_i2c.h"
#include "bsp_laser.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "fortress.h"
#include "gimbal.h"
#include "pose.h"
#include "protocol.h"
#include "shooter.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;

static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;
static control::MotorCANBase* sl_motor = nullptr;
static control::MotorCANBase* sr_motor = nullptr;
static control::MotorCANBase* ld_motor = nullptr;
static control::Shooter* shooter = nullptr;
static remote::DBUS* dbus = nullptr;
static bsp::Laser* laser = nullptr;
static bsp::WT901* imu_pitch = nullptr;
static bsp::MPU6500* imu_yaw = nullptr;
static control::Pose* poseEstimator = nullptr;
static control::MotorCANBase* fortress_motor0 = nullptr;
static control::MotorCANBase* fortress_motor1 = nullptr;
static control::MotorCANBase* fortress_yaw_motor = nullptr;
static control::Fortress* fortress = nullptr;

const uint16_t IMUAddress = 0x50;
static bsp::GPIO *gpio_red, *gpio_green;

static volatile bool spin_mode = false;

const int IMU_TASK_DELAY = 2;
const int GIMBAL_TASK_DELAY = 5;
const int CHASSIS_TASK_DELAY = 10;
const float CHASSIS_DEADZONE = 0.08;
const int SHOOTER_TASK_DELAY = 10;
const int FORTRESS_TASK_DELAY = 10;

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

volatile float ch0;
volatile float ch1;
volatile float ch2;
volatile float ch3;

control::gimbal_data_t* gimbal_param = nullptr;

#define RX_SIGNAL (1 << 0)

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

const osThreadAttr_t fortressTaskAttribute = {.name = "fortressTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 128 * 4,
                                              .priority = (osPriority_t)osPriorityNormal,
                                              .tz_module = 0,
                                              .reserved = 0};
osThreadId_t fortressTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL); }
};

communication::Referee* referee = nullptr;
CustomUART* referee_uart = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim2);

  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);

  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can2, 0x206);

  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_STANDARD_2022_ALPHA;
  gimbal = new control::Gimbal(gimbal_data);
  gimbal_param = gimbal->GetData();

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

  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);

  referee_uart = new CustomUART(&huart7);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  bsp::GPIO chip_select(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
  imu_yaw = new bsp::MPU6500(&ONBOARD_IMU_SPI, chip_select, MPU6500_IT_Pin);
  poseEstimator = new control::Pose(imu_yaw);
  imu_pitch = new bsp::WT901(&hi2c2, IMUAddress);

  fortress_motor0 = new control::Motor3508(can2, 0x207);
  fortress_motor1 = new control::Motor3508(can2, 0x208);
  control::MotorCANBase* fortress_motors[FORTRESS_MOTOR_NUM];
  fortress_motors[0] = fortress_motor0;
  fortress_motors[1] = fortress_motor1;
  fortress_yaw_motor = new control::Motor6020(can2, 0x205);
  fortress = new control::Fortress(fortress_motors, fortress_yaw_motor);
}

void KillAll() {
  control::MotorCANBase* motors_can1_pitch[] = {pitch_motor};
  control::MotorCANBase* motors_can2_yaw[] = {yaw_motor};
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

  RM_EXPECT_TRUE(false, "Operation killed\r\n");
  while (true) {
    if (dbus->keyboard.bit.V) {
      break;
    }
    fl_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    br_motor->SetOutput(0);
    pitch_motor->SetOutput(0);
    yaw_motor->SetOutput(0);
    sl_motor->SetOutput(0);
    sr_motor->SetOutput(0);
    ld_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
    laser->Off();
    osDelay(10);
  }
}

volatile bool imu_inited = false;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  osDelay(3000);
  print("IMU Initialized!\r\n");

  poseEstimator->SetAlpha(0.95);
  poseEstimator->SetGravityDir(control::Pose::X);

  // calibrate the Offset for IMU acce meter and gyro
  poseEstimator->Calibrate();

  // reset timer and pose for IMU
  poseEstimator->PoseInit();

  imu_inited = true;
  print("Pose Initialized!\r\n");

  while (true) {
    poseEstimator->ComplementaryFilterUpdate();
    osDelay(IMU_TASK_DELAY);
  }
}

void gimbalTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_pitch[] = {pitch_motor};
  control::MotorCANBase* motors_can2_yaw[] = {yaw_motor};

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  float angle[3];
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;

  print("Calibrate\r\n");
  gpio_red->Low();
  gpio_green->Low();
  for (int i = 0; i < 800; i++) {
    //  while (true) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);
    if (dbus->keyboard.bit.B) KillAll();
    osDelay(5);
  }
  gpio_red->High();
  gpio_green->High();

  while (!imu_inited) {
    osDelay(100);
  }
  laser->Off();
  print("Gimbal Begin\r\n");

  bool i2c_error_flag = false;

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) {
          break;
        }
        osDelay(10);
      }
    }

    if (!(imu_pitch->GetAngle(angle))) {
      RM_ASSERT_TRUE(false, "I2C Error!\r\n");
      i2c_error_flag = true;
    } else {
      i2c_error_flag = false;
    }
    angle[2] = poseEstimator->GetYaw();

    float pitch_ratio = -dbus->mouse.y / 32767.0 * 7.5;
    float yaw_ratio = -dbus->mouse.x / 32767.0 * 7.5;
    pitch_ratio += ch3 / 18000.0;
    yaw_ratio += -ch2 / 18000.0;
    pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                               gimbal_param->pitch_max_);
    //    if (!spin_mode)
    //      yaw_target = clip<float>(yaw_target + yaw_ratio / 30.0, -2.8, 2.8);
    //    else
    yaw_target = wrap<float>(yaw_target + yaw_ratio, -PI, PI);

    pitch_curr = -angle[1];
    yaw_curr = angle[2];
    float pitch_diff = wrap<float>(pitch_target - pitch_curr, -PI, PI);
    float yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);
    // print("Y_CURR: %6.3f, Y_DIFF: %6.3f\r\n", yaw_curr, yaw_diff);
    if (!i2c_error_flag) {
      gimbal->TargetRel(pitch_diff / 28, yaw_diff / 33);
    } else {
      gimbal->TargetRel(0, yaw_diff / 33);
    }

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_pitch, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_yaw, 1);

    osDelay(GIMBAL_TASK_DELAY);
  }
}

void chassisTask(void* arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  osDelay(5000);
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

  float sin_yaw, cos_yaw;
  float vx = 0, vy = 0, wz = 0;
  float vx_remote = 0, vy_remote = 0;
  float vx_set, vy_set, wz_set;
  float relative_angle;

  float spin_speed = 650;
  float follow_speed = 650;

  bool follow_mode = true;
  int mode_change_delay = 200;
  int mode_change_count = 0;
  //  bool prev_mode = true;

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      chassis->SetSpeed(0, 0, 0);
      chassis->Update(referee->power_heat_data.chassis_power,
                      referee->power_heat_data.chassis_power_buffer);
      while (true) {
        if (dbus->keyboard.bit.V) {
          break;
        }
        osDelay(10);
      }
    }

    vx_remote = ch0;
    vy_remote = ch1;

    if (dbus->keyboard.bit.SHIFT || dbus->swl == remote::UP) {
      if (mode_change_count > mode_change_delay) {
        mode_change_count = 0;
        follow_mode = !follow_mode;
      }
    }
    mode_change_count++;

    if (dbus->keyboard.bit.A) vx -= 80;
    if (dbus->keyboard.bit.D) vx += 80;
    if (dbus->keyboard.bit.W) vy += 80;
    if (dbus->keyboard.bit.S) vy -= 80;

    if (-35 <= vx && vx <= 35) {
      vx = 0;
    }
    if (-35 <= vy && vy <= 35) {
      vy = 0;
    }

    if (vx > 0) {
      vx -= 60;
    } else if (vx < 0) {
      vx += 60;
    }

    if (vy > 0) {
      vy -= 60;
    } else if (vy < 0) {
      vy += 60;
    }

    vx = clip<float>(vx, -1200, 1200);
    vy = clip<float>(vy, -1200, 1200);
    UNUSED(wz);
    relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
    //    relative_angle = 0;
    if (relative_angle < CHASSIS_DEADZONE && relative_angle > -CHASSIS_DEADZONE) relative_angle = 0;
    sin_yaw = arm_sin_f32(relative_angle);
    cos_yaw = arm_cos_f32(relative_angle);
    vx_set = cos_yaw * (vx + vx_remote) + sin_yaw * (vy + vy_remote);
    vy_set = -sin_yaw * (vx + vx_remote) + cos_yaw * (vy + vy_remote);
    if (!follow_mode) {
      spin_mode = true;
      wz_set = spin_speed;
    } else {
      //      if (spin_mode) {
      //        float rotate = follow_speed * relative_angle;
      //        wz_set = abs(rotate < spin_speed ? rotate : spin_speed);
      //      } else {
      wz_set = follow_speed * relative_angle;
      //      }
      //      if (relative_angle == 0 && !dbus->keyboard.bit.SHIFT) {
      //        spin_mode = false;
      //      }
      wz_set = clip<float>(wz_set, -290, 290);
    }
    chassis->SetSpeed(vx_set, vy_set, wz_set);
    //    print("chassis\r\n");
    chassis->Update(referee->power_heat_data.chassis_power,
                    referee->power_heat_data.chassis_power_buffer);
    UNUSED(motors_can2_chassis);
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

    osDelay(CHASSIS_TASK_DELAY);
  }
}

void shooterTask(void* arg) {
  UNUSED(arg);
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  while (true) {
    if (dbus->keyboard.bit.CTRL || dbus->swr == remote::DOWN) {
      //           for (int i = 0; i < 50; ++i) {
      //               shooter->SetFlywheelSpeed(0);
      //                shooter->Update();
      //                control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
      //               osDelay(10);
      //           }
      break;
    }
    osDelay(100);
  }

  //  for (int i = 0; i <=450; i = i + 1) {
  //      shooter->SetFlywheelSpeed(i);
  //      shooter->Update();
  //      control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
  //      osDelay(1);
  //  }

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) {
          break;
        }
        osDelay(10);
      }
    }
    //    if (referee->game_robot_status.mains_power_shooter_output && dbus->mouse.l) {
    //        shooter->LoadNext();
    //    }
    //    if (referee->game_robot_status.mains_power_shooter_output && dbus->swr == remote::UP) {
    //        shooter->LoadNext();
    //    }
    //    if (!referee->game_robot_status.mains_power_shooter_output || dbus->keyboard.bit.Q ||
    //    dbus->swr == remote::DOWN) {
    //        shooter->SetFlywheelSpeed(0);
    //    } else {
    //        shooter->SetFlywheelSpeed(450);
    //    }
    if (dbus->swr == remote::UP) {
      shooter->LoadNext();
    }
    shooter->SetFlywheelSpeed(0);
    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
    osDelay(SHOOTER_TASK_DELAY);
  }
}

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

void fortressTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_fortress_rise[] = {fortress_motor0, fortress_motor1};
  control::MotorCANBase* motors_fortress_yaw[] = {fortress_yaw_motor};

  bool rise_state = false;
  int transform_time = 1000;

  while (dbus->swr != remote::DOWN) {
    if (dbus->swr == remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  while (dbus->swr == remote::DOWN) {
    if (dbus->swr != remote::DOWN) {
      break;
    }
    osDelay(100);
  }

  while (true) {
    if (dbus->swr == remote::DOWN) {
      if (!rise_state) {
        fortress->Up();
        osDelay(transform_time);
        fortress->StopRise();
        fortress->Rotate();
        rise_state = true;
      }
    } else {
      if (rise_state) {
        fortress->StopRotate();
        fortress->Down();
        osDelay(transform_time);
        fortress->StopRise();
        rise_state = false;
      }
    }
    control::MotorCANBase::TransmitOutput(motors_fortress_rise, FORTRESS_MOTOR_NUM);
    control::MotorCANBase::TransmitOutput(motors_fortress_yaw, 1);
    osDelay(FORTRESS_TASK_DELAY);
  }
}

void RM_RTOS_Threads_Init(void) {
  osDelay(500);  // DBUS initialization needs time
                 //  while (dbus->swr != remote::DOWN) {
                 //    if (dbus->swr == remote::DOWN) {
                 //      break;
                 //    }
                 //    osDelay(100);
                 //  }
                 //  print("test\r\n");
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  shooterTaskHandle = osThreadNew(shooterTask, nullptr, &shooterTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  //  fortressTaskHandle = osThreadNew(fortressTask, nullptr, &fortressTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  FirstOrderFilter f0(0.4);
  FirstOrderFilter f1(0.4);
  FirstOrderFilter f2(0.4);
  FirstOrderFilter f3(0.4);

  //  while (dbus->swr != remote::DOWN) {
  //    if (dbus->swr == remote::DOWN) {
  //      break;
  //    }
  //    osDelay(100);
  //  }

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) KillAll();
    //    print("Hello\r\n");
    ch0 = f0.CalculateOutput(dbus->ch0);
    ch1 = f1.CalculateOutput(dbus->ch1);
    ch2 = f2.CalculateOutput(dbus->ch2);
    ch3 = f3.CalculateOutput(dbus->ch3);

    //    set_cursor(0, 0);
    //    clear_screen();
    //    print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
    //    print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
    //    print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
    //    print("Chassis Power Buffer: %.3f\r\n", referee->power_heat_data.chassis_power_buffer);
    //    print("\r\n");
    //    print("Shooter Cooling Heat: %hu\r\n",
    //    referee->power_heat_data.shooter_id1_17mm_cooling_heat); print("Bullet Frequency:
    //    %hhu\r\n", referee->shoot_data.bullet_freq); print("Bullet Speed: %.3f\r\n",
    //    referee->shoot_data.bullet_speed);

    osDelay(100);
  }
}
