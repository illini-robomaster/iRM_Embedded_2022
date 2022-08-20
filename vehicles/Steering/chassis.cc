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

#include "bsp_can_bridge.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "steering.h"
#include "user_interface.h"

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static display::RGB* RGB = nullptr;

static BoolEdgeDetector FakeDeath(false);
static volatile bool Dead = false;
static BoolEdgeDetector ChangeSpinMode(false);
static volatile bool SpinMode = false;

static bsp::CanBridge* receive = nullptr;

static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int CHASSIS_TASK_DELAY = 2;
static const int UI_TASK_DELAY = 20;

//==================================================================================================
// Referee
//==================================================================================================

#define REFEREE_RX_SIGNAL (1 << 1)

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 1024 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

//==================================================================================================
// Chassis
//==================================================================================================

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

static control::MotorCANBase* motor1 = nullptr;
static control::MotorCANBase* motor2 = nullptr;
static control::MotorCANBase* motor3 = nullptr;
static control::MotorCANBase* motor4 = nullptr;
static control::MotorCANBase* motor5 = nullptr;
static control::MotorCANBase* motor6 = nullptr;
static control::MotorCANBase* motor7 = nullptr;
static control::MotorCANBase* motor8 = nullptr;

static bsp::GPIO* key1 = nullptr;
static bsp::GPIO* key2 = nullptr;
static bsp::GPIO* key3 = nullptr;
static bsp::GPIO* key4 = nullptr;

static control::steering_chassis_t* chassis_data;
static control::SteeringChassis* chassis;

static const float CHASSIS_DEADZONE = 0.04;

bool steering_align_detect1() { return !key1->Read(); }

bool steering_align_detect2() { return !key2->Read(); }

bool steering_align_detect3() { return !key3->Read(); }

bool steering_align_detect4() { return !key4->Read(); }

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};
  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

  float spin_speed = 10;
  float follow_speed = 10;

  while (!receive->start) osDelay(100);

  while (receive->start < 0.5) osDelay(100);

  while (true) {
    float relative_angle = receive->relative_angle;
    float sin_yaw, cos_yaw, vx_set, vy_set, wz_set;
    UNUSED(wz_set);

    vx_set = receive->vx;
    vy_set = receive->vy;

    if (receive->mode == 1) {  // spin mode
      sin_yaw = arm_sin_f32(relative_angle);
      cos_yaw = arm_cos_f32(relative_angle);
      vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
      vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
      wz_set = spin_speed;
    } else {
      sin_yaw = arm_sin_f32(relative_angle);
      cos_yaw = arm_cos_f32(relative_angle);
      vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
      vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
      wz_set = std::min(follow_speed, follow_speed * relative_angle);
      if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz_set = 0;
    }

    chassis->SetYSpeed(-vx_set / 10);
    chassis->SetXSpeed(-vy_set / 10);
    chassis->SetWSpeed(wz_set);
    chassis->Update((float)referee->game_robot_status.chassis_power_limit,
                    referee->power_heat_data.chassis_power,
                    (float)referee->power_heat_data.chassis_power_buffer);

    if (Dead) {
      motor5->SetOutput(0);
      motor6->SetOutput(0);
      motor7->SetOutput(0);
      motor8->SetOutput(0);
    }

    control::MotorCANBase::TransmitOutput(wheel_motors, 4);
    control::MotorCANBase::TransmitOutput(steer_motors, 4);

    receive->cmd.id = bsp::SHOOTER_POWER;
    receive->cmd.data_bool = referee->game_robot_status.mains_power_shooter_output;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_HEAT1;
    receive->cmd.data_float = (float)referee->power_heat_data.shooter_id1_17mm_cooling_heat;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_HEAT2;
    receive->cmd.data_float = (float)referee->power_heat_data.shooter_id2_17mm_cooling_heat;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_LIMIT1;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_cooling_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_LIMIT2;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_cooling_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::SPEED_LIMIT1;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_speed_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::SPEED_LIMIT2;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_speed_limit;
    receive->TransmitOutput();

    osDelay(CHASSIS_TASK_DELAY);
  }
}

//==================================================================================================
// UI
//==================================================================================================

const osThreadAttr_t UITaskAttribute = {.name = "UITask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 1024 * 4,
                                        .priority = (osPriority_t)osPriorityBelowNormal,
                                        .tz_module = 0,
                                        .reserved = 0};

osThreadId_t UITaskHandle;

// static distance::LIDAR07_UART* LIDAR = nullptr;
static communication::UserInterface* UI = nullptr;

void UITask(void* arg) {
  UNUSED(arg);

//  while (!selftestStart) osDelay(100);

  //   int tryLIDAR = 0;
  //   while (!LIDAR->begin()) {
  //     if (++tryLIDAR >= 5) break;
  //     osDelay(10);
  //   }
  //   tryLIDAR = 0;
  //   while (!LIDAR->startFilter()) {
  //     if (++tryLIDAR >= 5) break;
  //     osDelay(10);
  //   }

  UI->SetID(referee->game_robot_status.robot_id);

  communication::package_t frame;
  communication::graphic_data_t graphGimbal;
  communication::graphic_data_t graphChassis;
  communication::graphic_data_t graphArrow;
  communication::graphic_data_t graphCali;
  communication::graphic_data_t graphEmpty2;
  communication::graphic_data_t graphCrosshair1;
  communication::graphic_data_t graphCrosshair2;
  communication::graphic_data_t graphCrosshair3;
  communication::graphic_data_t graphCrosshair4;
  communication::graphic_data_t graphCrosshair5;
  communication::graphic_data_t graphCrosshair6;
  communication::graphic_data_t graphCrosshair7;
  communication::graphic_data_t graphBarFrame;
  communication::graphic_data_t graphBar;
  communication::graphic_data_t graphPercent;
  communication::graphic_data_t graphDiag;
  communication::graphic_data_t graphMode;
  communication::graphic_data_t graphDist;
  //  communication::graphic_data_t graphLid;
  communication::graphic_data_t graphWheel;

//  char msgBuffer1[30] = "PITCH MOTOR UNCONNECTED";
//  char msgBuffer2[30] = "YAW MOTOR UNCONNECTED";
//  char msgBuffer3[30] = "L SHOOTER MOTOR UNCONNECTED";
//  char msgBuffer4[30] = "R SHOOTER MOTOR UNCONNECTED";
//  char msgBuffer5[30] = "LOAD MOTOR UNCONNECTED";
//  char msgBuffer6[30] = "FRONT L MOTOR UNCONNECTED";
//  char msgBuffer7[30] = "FRONT R MOTOR UNCONNECTED";
//  char msgBuffer8[30] = "BACK L MOTOR UNCONNECTED";
//  char msgBuffer9[30] = "BACK R MOTOR UNCONNECTED";

//  bool pitch_motor_flag_ui = pitch_motor_flag;
//  bool yaw_motor_flag_ui = yaw_motor_flag;
//  bool sl_motor_flag_ui = sl_motor_flag;
//  bool sr_motor_flag_ui = sr_motor_flag;
//  bool ld_motor_flag_ui = ld_motor_flag;
//  bool fl_motor_flag_ui = fl_motor_flag;
//  bool fr_motor_flag_ui = fr_motor_flag;
//  bool bl_motor_flag_ui = bl_motor_flag;
//  bool br_motor_flag_ui = br_motor_flag;

  // Initialize chassis GUI
  UI->ChassisGUIInit(&graphChassis, &graphArrow, &graphGimbal, &graphCali, &graphEmpty2);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                   graphCali, graphEmpty2);
  referee->PrepareUIContent(communication::FIVE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize crosshair GUI
  UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                   &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2,
                   graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                   graphCrosshair7);
  referee->PrepareUIContent(communication::SEVEN_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize supercapacitor GUI
  UI->CapGUIInit(&graphBarFrame, &graphBar);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphBarFrame, graphBar);
  referee->PrepareUIContent(communication::DOUBLE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize Supercapacitor string GUI
  UI->CapGUICharInit(&graphPercent);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                  UI->getPercentLen());
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize self-diagnosis GUI
  char diagStr[30] = "";
  UI->DiagGUIInit(&graphDiag, 30);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDiag, diagStr, 2);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize current mode GUI
  char followModeStr[15] = "FOLLOW MODE";
  char spinModeStr[15] = "SPIN  MODE";
  uint32_t modeColor = UI_Color_Orange;
  UI->ModeGUIInit(&graphMode);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, followModeStr,
                  sizeof followModeStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize distance GUI
  char distanceStr[15] = "0.0";
  UI->DistanceGUIInit(&graphDist);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr,
                  sizeof distanceStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // TODO: add lid UI in the future

  //  // Initialize lid status GUI
  //  char lidOpenStr[15] = "LID OPENED";
  //  char lidCloseStr[15] = "LID CLOSED";
  //  UI->LidGUIInit(&graphLid);
  //  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidOpenStr, sizeof
  //  lidOpenStr); referee->PrepareUIContent(communication::CHAR_GRAPH); frame =
  //  referee->Transmit(communication::STUDENT_INTERACTIVE); referee_uart->Write(frame.data,
  //  frame.length); osDelay(UI_TASK_DELAY);

  // Initialize flywheel status GUI
//  char wheelOnStr[15] = "FLYWHEEL ON";
  char wheelOffStr[15] = "FLYWHEEL OFF";
  UI->WheelGUIInit(&graphWheel);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelOffStr,
                  sizeof wheelOffStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  float j = 1;
  while (true) {
    //     lidar_flag = LIDAR->startMeasure();
    float relative_angle = receive->relative_angle;
    // Update chassis GUI
    UI->ChassisGUIUpdate(relative_angle, false);
    UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                     graphCali, graphEmpty2);
    referee->PrepareUIContent(communication::FIVE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update supercapacitor GUI
    UI->CapGUIUpdate(std::abs(sin(j)));
    UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graphBar);
    referee->PrepareUIContent(communication::SINGLE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    j += 0.1;
    osDelay(UI_TASK_DELAY);

    // Update supercapacitor string GUI
    UI->CapGUICharUpdate();
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                    UI->getPercentLen());
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update current mode GUI
    char* modeStr = SpinMode ? spinModeStr : followModeStr;
    modeColor = SpinMode ? UI_Color_Green : UI_Color_Orange;
    UI->ModeGuiUpdate(&graphMode, modeColor);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, modeStr, 15);
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update distance GUI
    //     uint32_t distColor = UI_Color_Cyan;
    //     float currDist = LIDAR->distance / 1000.0;
    //     if (currDist < 60) {
    //       snprintf(distanceStr, 15, "%.2f m", currDist);
    //       distColor = UI_Color_Cyan;
    //     } else {
    //       snprintf(distanceStr, 15, "ERROR");
    //       distColor = UI_Color_Pink;
    //     }
    //     UI->DistanceGUIUpdate(&graphDist, distColor);
    //     UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr, 15);
    //     referee->PrepareUIContent(communication::CHAR_GRAPH);
    //     frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    //     referee_uart->Write(frame.data, frame.length);
    //     osDelay(UI_TASK_DELAY);

    //    // Update lid status GUI
    //    char lidStr[15] = lidFlag ? lidOpenStr : lidCloseStr;
    //    uint32_t lidColor = lidFlag ? UI_Color_Pink : UI_Color_Green;
    //    UI->LidGuiUpdate(&graphLid, lidColor);
    //    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidStr, 15);
    //    referee->PrepareUIContent(communication::CHAR_GRAPH);
    //    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    //    referee_uart->Write(frame.data, frame.length);
    //    osDelay(UI_TASK_DELAY);

//    // Update wheel status GUI
//    char* wheelStr = flywheelFlag ? wheelOnStr : wheelOffStr;
//    uint32_t wheelColor = flywheelFlag ? UI_Color_Pink : UI_Color_Green;
//    UI->WheelGUIUpdate(&graphWheel, wheelColor);
//    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelStr, 15);
//    referee->PrepareUIContent(communication::CHAR_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(UI_TASK_DELAY);
//
//    // Update self-diagnosis messages
//    if (!pitch_motor_flag_ui && !pitch_motor_flag) {
//      UI->AddMessage(msgBuffer1, sizeof msgBuffer1, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      pitch_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!yaw_motor_flag_ui && !yaw_motor_flag) {
//      UI->AddMessage(msgBuffer2, sizeof msgBuffer2, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      yaw_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!sl_motor_flag_ui && !sl_motor_flag) {
//      UI->AddMessage(msgBuffer3, sizeof msgBuffer3, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      sl_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!sr_motor_flag_ui && !sr_motor_flag) {
//      UI->AddMessage(msgBuffer4, sizeof msgBuffer4, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      sr_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!ld_motor_flag_ui && !ld_motor_flag) {
//      UI->AddMessage(msgBuffer5, sizeof msgBuffer5, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      ld_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!fl_motor_flag_ui && !fl_motor_flag) {
//      UI->AddMessage(msgBuffer6, sizeof msgBuffer6, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      fl_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!fr_motor_flag_ui && !fr_motor_flag) {
//      UI->AddMessage(msgBuffer7, sizeof msgBuffer7, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      fr_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!bl_motor_flag_ui && !bl_motor_flag) {
//      UI->AddMessage(msgBuffer8, sizeof msgBuffer8, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      bl_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!br_motor_flag_ui && !br_motor_flag) {
//      UI->AddMessage(msgBuffer9, sizeof msgBuffer9, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      br_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    // clear self-diagnosis messages
//    if (dbus->keyboard.bit.C) {
//      for (int i = 1; i <= UI->getMessageCount(); ++i) {
//        UI->DiagGUIClear(UI, referee, &graphDiag, i);
//        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//        referee_uart->Write(frame.data, frame.length);
//        osDelay(UI_TASK_DELAY);
//      }
//    }
  }
}

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

  motor1 = new control::Motor3508(can1, 0x201);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  motor5 = new control::Motor3508(can2, 0x205);
  motor6 = new control::Motor3508(can2, 0x206);
  motor7 = new control::Motor3508(can2, 0x207);
  motor8 = new control::Motor3508(can2, 0x208);

  key1 = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  key2 = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  key3 = new bsp::GPIO(IN3_GPIO_Port, IN3_Pin);
  key4 = new bsp::GPIO(IN4_GPIO_Port, IN4_Pin);

  chassis_data = new control::steering_chassis_t();

  chassis_data->fl_steer_motor = motor4;
  chassis_data->fr_steer_motor = motor3;
  chassis_data->bl_steer_motor = motor1;
  chassis_data->br_steer_motor = motor2;

  chassis_data->fl_steer_motor_detect_func = steering_align_detect4;
  chassis_data->fr_steer_motor_detect_func = steering_align_detect3;
  chassis_data->bl_steer_motor_detect_func = steering_align_detect1;
  chassis_data->br_steer_motor_detect_func = steering_align_detect2;

  // TODO init wheels
  chassis_data->fl_wheel_motor = motor8;
  chassis_data->fr_wheel_motor = motor7;
  chassis_data->bl_wheel_motor = motor5;
  chassis_data->br_wheel_motor = motor6;

  chassis = new control::SteeringChassis(chassis_data);

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  receive = new bsp::CanBridge(can2, 0x20B, 0x20A);

  UI = new communication::UserInterface();
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  UITaskHandle = osThreadNew(UITask, nullptr, &UITaskAttribute);
}

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

  RGB->Display(display::color_blue);

  while (true) {
    if (!receive->dead) {
      SpinMode = false;
      Dead = false;
      RGB->Display(display::color_green);
      break;
    }

    motor5->SetOutput(0);
    motor6->SetOutput(0);
    motor7->SetOutput(0);
    motor8->SetOutput(0);

    control::MotorCANBase::TransmitOutput(wheel_motors, 4);

    osDelay(KILLALL_DELAY);
  }
}

static bool debug = false;

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    if (receive->dead) {
      Dead = true;
      KillAll();
    }
    if (debug) {
      set_cursor(0, 0);
      clear_screen();
      print("vx: %f, vy: %f, angle: %f, mode: %f, dead: %f\r\n", receive->vx, receive->vy,
            receive->relative_angle, receive->mode, receive->dead);
    }
    osDelay(DEFAULT_TASK_DELAY);
  }
}
