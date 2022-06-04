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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "user_interface.h"
#include <cmath>

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
  print_use_uart(&huart8);

  UI = new communication::UserInterface(UI_Data_RobotID_RStandard3, UI_Data_CilentID_RStandard3);

  referee_uart = new CustomUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);

  referee = new communication::Referee;
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  communication::package_t frame;

  communication::graphic_data_t graphGimbal;
  communication::graphic_data_t graphChassis;
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

//  communication::graphic_data_t graph2;
//  communication::graphic_data_t graph2;
//  UI->RectangleDraw(&graph1, "0", UI_Graph_Add, 0, UI_Color_Purplish_red, 5, 960, 540, 1000, 700);
//UI->LineDraw(&graph2, "0", UI_Graph_Add, 0, UI_Color_Orange, 100, 960, 540, 1300, 540);
//  UI->CircleDraw(&graph2, "1", UI_Graph_Add, 1, UI_Color_Cyan, 5, 960, 540, 50);
//  UI->ArcDraw(&graph, "0", UI_Graph_Add, 0, UI_Color_Cyan, 0, 200, 5, 960, 540, 50, 150);
//  UI->EllipseDraw(&graph, "0", UI_Graph_Add, 0, UI_Color_Cyan, 10, 960, 540, 50, 150);
//UI->IntDraw(&graph1, "0", UI_Graph_Add, 0, UI_Color_Cyan, 30, 5, 960, 540, -876876586);
//UI->FloatDraw(&graph1, "0", UI_Graph_Add, 0, UI_Color_Cyan, 30, 3, 3, 960, 540, 3.567);
//  UI->LineDraw(&graph2, "0", UI_Graph_Add, 0, UI_Color_Orange, 100, 960, 540, 1300, 540);

  UI->ChassisGUIInit(&graphGimbal, &graphChassis, 1300, 150);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphChassis, graphGimbal);
  referee->PrepareUIContent(communication::DOUBLE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4, &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2, graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6, graphCrosshair7);
  referee->PrepareUIContent(communication::SEVEN_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

  UI->SuperCapGUIInit(&graphBarFrame, &graphBar, &graphPercent, 1500, 350);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphBarFrame, graphBar);
  referee->PrepareUIContent(communication::DOUBLE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);

//  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, , );

  referee_uart->Write(frame.data, frame.length);
  osDelay(100);

//  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, percentString, );
//  referee->PrepareUIContent(communication::CHAR_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(100);

  float i = 0;
  float j = 1.0;
  while (true){
      UI->ChassisGUIUpdate(i);
      UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphChassis, graphGimbal);
      referee->PrepareUIContent(communication::DOUBLE_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      i+=0.1;
      osDelay(50);

      UI->SuperCapGUIUpdate(std::abs(sin(j)));
      UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graphBar);
      referee->PrepareUIContent(communication::SINGLE_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      j+=0.1;
      osDelay(50);

//      UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, percentString, );
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      osDelay(100);
  }

//char theString[30];
//int length = snprintf(theString, 30, "%.10f", -3.1415926);
//UI->CharDraw(&graph1, "1", UI_Graph_Add, 0, UI_Color_Pink, 30, length, 3, 960, 540);
////UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graph1);
//UI->CharRefresh((uint8_t*)(&referee->graphic_character), graph1, theString, length);
//  referee->PrepareUIContent(communication::CHAR_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(100);

//  int i = 0;
//  while (true) {
//    UI->RectangleDraw(&graph, "0", UI_Graph_Change, 0, UI_Color_Purplish_red, 10, 960 - i, 640, 1000, 700);
//    UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graph);
//    referee->PrepareUIContent(communication::SINGLE_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(100);
//    ++i;
//  }
}
