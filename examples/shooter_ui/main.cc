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
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "user_interface.h"
#include "bsp_gpio.h"
#include "bsp_laser.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "shooter.h"
#include <cmath>

#define RX_SIGNAL (1 << 0)
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG

bsp::CAN* can = nullptr;
control::MotorCANBase* left_flywheel_motor = nullptr;
control::MotorCANBase* right_flywheel_motor = nullptr;
control::MotorCANBase* load_motor = nullptr;

remote::DBUS* dbus = nullptr;
control::ServoMotor* load_servo = nullptr;
control::Shooter* shooter = nullptr;

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

static communication::UserInterface* UI = nullptr;
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

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);

  UI = new communication::UserInterface(UI_Data_RobotID_RStandard3, UI_Data_CilentID_RStandard3);

  referee_uart = new CustomUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);

  referee = new communication::Referee;

  dbus = new remote::DBUS(&huart3);

  can = new bsp::CAN(&hcan1, 0x201);
  left_flywheel_motor = new control::Motor3508(can, 0x201);
  right_flywheel_motor = new control::Motor3508(can, 0x202);
  load_motor = new control::Motor3508(can, 0x203);

  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = left_flywheel_motor;
  shooter_data.right_flywheel_motor = right_flywheel_motor;
  shooter_data.load_motor = load_motor;
  shooter_data.model = control::SHOOTER_STANDARD_2022;
  shooter = new control::Shooter(shooter_data);
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[] = {left_flywheel_motor, right_flywheel_motor, load_motor};
  bsp::GPIO laser(LASER_GPIO_Port, LASER_Pin);
  laser.High();

  communication::package_t frame;
  communication::graphic_data_t graphGimbal;
  communication::graphic_data_t graphChassis;
  communication::graphic_data_t graphArrow;
  communication::graphic_data_t graphEmpty1;
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

  UI->ChassisGUIInit(&graphChassis, &graphArrow, &graphGimbal, &graphEmpty1, &graphEmpty2);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal, graphEmpty1, graphEmpty2);
  referee->PrepareUIContent(communication::FIVE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4, &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2, graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6, graphCrosshair7);
  referee->PrepareUIContent(communication::SEVEN_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  UI->CapGUIInit(&graphBarFrame, &graphBar);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphBarFrame, graphBar);
  referee->PrepareUIContent(communication::DOUBLE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  UI->CapGUICharInit(&graphPercent);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(), UI->getPercentLen());
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  char diagStr[30] = " ";
  UI->DiagGUIInit(&graphDiag, 30);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDiag, diagStr, 2);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  char msgBuffer[30] = "Error_one";
  UI->AddMessage(msgBuffer, sizeof msgBuffer, UI, referee, &graphDiag);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  char msgBuffer2[30] = "Error_two";
  UI->AddMessage(msgBuffer2, sizeof msgBuffer2, UI, referee, &graphDiag);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  char msgBuffer3[30] = "Error_three";
  UI->AddMessage(msgBuffer3, sizeof msgBuffer3, UI, referee, &graphDiag);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  char str[] = "NORMAL MODE";
  UI->ModeGUIInit(&graphMode, sizeof str - 1);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, str, sizeof str);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  osDelay(2000);
  char modeStr[] = "SPIN MODE";     // mode name
  UI->ModeGuiUpdate(&graphMode, sizeof modeStr - 1);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, modeStr, sizeof modeStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);


//  osDelay(2000);
////   clear diagnosis messages
//  for (int i = 1; i <= UI->getMessageCount(); ++i){
//    UI->DiagGUIClear(UI, referee, &graphDiag, i);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(50);
//  }


  float i = 0;
  float j = 1.0;
  while (true){
      UI->ChassisGUIUpdate(i);
      UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal, graphEmpty1, graphEmpty2);
      referee->PrepareUIContent(communication::FIVE_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      i+=0.1;

      UI->CapGUIUpdate(std::abs(sin(j)));
      UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graphBar);
      referee->PrepareUIContent(communication::SINGLE_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      j+=0.1;

      UI->CapGUICharUpdate();
      UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(), UI->getPercentLen());
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      osDelay(100);

      if (dbus->swr == remote::DOWN) shooter->SetFlywheelSpeed(500);
      if (dbus->swr == remote::UP) shooter->SetFlywheelSpeed(0);
      if (dbus->ch3 > 500) shooter->LoadNext();
      shooter->Update();
      control::MotorCANBase::TransmitOutput(motors, 3);
      osDelay(10);
  }

}