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

#include "protocol.h"

/********************** content ID data********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/********************** red ID ****************************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/********************** blue ID ***************************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/********************** red operator ID *******************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/********************** blue operator ID ******************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/********************** deletion **************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/********************** operations ************************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/********************** graph configurations - types ******/
#define UI_Graph_Line 0
#define UI_Graph_Rectangle 1
#define UI_Graph_Circle 2
#define UI_Graph_Ellipse 3
#define UI_Graph_Arc 4
#define UI_Graph_Float 5
#define UI_Graph_Int 6
#define UI_Graph_Char 7
/********************** graph configurations - colors *****/
#define UI_Color_Main 0
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4
#define UI_Color_Pink 5
#define UI_Color_Cyan 6
#define UI_Color_Black 7
#define UI_Color_White 8

namespace communication {

class UserInterface {
 public:
  UserInterface(int Robot_ID, int Client_ID);
  void LineDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
  void RectangleDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
  void CircleDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t graph_radius);
  void EllipseDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t x_length, uint32_t y_length);
  void ArcDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_startAngle, uint32_t graph_endAngle, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t x_length, uint32_t y_length);
  void FloatDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width, uint32_t start_x, uint32_t start_y, float graph_float);
  void IntDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t graph_width, uint32_t start_x, uint32_t start_y, int graph_int);
  void CharDraw(graphic_data_t *image, const char name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t char_length, uint32_t graph_width, uint32_t start_x, uint32_t start_y);

  int UIDelete(uint8_t* data_buffer, uint8_t del_operate, uint8_t del_layer);
  int GraphRefresh(uint8_t* data_buffer, int cnt, ...);
  int CharRefresh(uint8_t* data_buffer, graphic_data_t image, char* theString, int len);
 private:
  int Robot_ID_;
  int Client_ID_;
};

}
