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

#include "user_interface.h"

#include <cstdarg>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <string>

namespace communication {

UserInterface::UserInterface(int Robot_ID, int Client_ID) {
  Robot_ID_ = Robot_ID;
  Client_ID_ = Client_ID;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graphOperate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param end_x ending x coordinate
 * @param end_y ending y coordinate
 */
void UserInterface::LineDraw(graphic_data_t* image, const char name[3], uint32_t graphOperate,
                             uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                             uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graphOperate;
  image->graphic_type = UI_Graph_Line;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = end_x;
  image->end_y = end_y;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param end_x ending x coordinate
 * @param end_y ending y coordinate
 */
void UserInterface::RectangleDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                  uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                                  uint32_t start_x, uint32_t start_y, uint32_t end_x,
                                  uint32_t end_y) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Rectangle;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = end_x;
  image->end_y = end_y;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param graph_radius graph radius
 */
void UserInterface::CircleDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                               uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                               uint32_t start_x, uint32_t start_y, uint32_t graph_radius) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Circle;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->radius = graph_radius;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting (center) x coordinate
 * @param start_y starting (center) y coordinate
 * @param x_length horizontal semi-axis length
 * @param y_length vertical semi-axis length
 */
void UserInterface::EllipseDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                                uint32_t start_x, uint32_t start_y, uint32_t x_length,
                                uint32_t y_length) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Ellipse;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = x_length;
  image->end_y = y_length;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_startAngle starting angle
 * @param graph_endAngle ending angle
 * @param graph_width graph width
 * @param start_x starting (center) x coordinate
 * @param start_y starting (center) y coordinate
 * @param x_length horizontal semi-axis length
 * @param y_length vertical semi-axis length
 */
void UserInterface::ArcDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                            uint32_t graph_layer, uint32_t graph_color, uint32_t graph_startAngle,
                            uint32_t graph_endAngle, uint32_t graph_width, uint32_t start_x,
                            uint32_t start_y, uint32_t x_length, uint32_t y_length) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Arc;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_startAngle;
  image->end_angle = graph_endAngle;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = x_length;
  image->end_y = y_length;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_size graph size
 * @param graph_digit number of digits after decimal
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param graph_float float to be displayed
 */
void UserInterface::FloatDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                              uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                              uint32_t graph_digit, uint32_t graph_width, uint32_t start_x,
                              uint32_t start_y, float graph_float) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Float;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_size;
  image->end_angle = graph_digit;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  float float_data = graph_float;
  int32_t float2int_data;
  memcpy(&float2int_data, (uint8_t*)&float_data, 4);
  image->radius = float2int_data & 0x3FF;
  image->end_x = (float2int_data & 0x1FFC00) >> 10;
  image->end_y = (float2int_data & 0xFFE00000) >> 21;
//  image->radius = (float2int_data & 0xFFC00000) >> 22;
//  image->end_x = (float2int_data & 0x003FF800) >> 11;
//  image->end_y = float2int_data & 0x000007FF;
}

void UserInterface::IntDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                            uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                            uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                            int graph_int) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Int;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_size;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  int32_t int_data = graph_int;
  image->radius = int_data & 0x3FF;
  image->end_x = (int_data & 0x1FFC00) >> 10;
  image->end_y = (int_data & 0xFFE00000) >> 21;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_size graph size
 * @param char_length character length
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 */
void UserInterface::CharDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                             uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                             uint32_t char_length, uint32_t graph_width, uint32_t start_x,
                             uint32_t start_y) {
  for (int i = 0; i < 3 && name[i] != 0; i++) {
    image->graphic_name[2 - i] = name[i];
  }
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Char;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_size;
  image->end_angle = char_length;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
}

int UserInterface::UIDelete(uint8_t* data_buffer, uint8_t del_operate, uint8_t del_layer) {
  graphic_delete_t del;
  del.header.data_cmd_id = UI_Data_ID_Del;
  del.header.sender_ID = Robot_ID_;
  del.header.receiver_ID = Client_ID_;
  del.operate_type = del_operate;
  del.layer = del_layer;
  int length = sizeof(graphic_delete_t);
  memcpy(data_buffer, &del, length);
  return length;
}

int UserInterface::GraphRefresh(uint8_t* data_buffer, int cnt, ...) {
  va_list arg;
  va_start(arg, cnt);
  UI_header_data_t header;
  header.sender_ID = Robot_ID_;
  header.receiver_ID = Client_ID_;
  int length = -1;
  switch (cnt) {
    case 1: {
      header.data_cmd_id = UI_Data_ID_Draw1;
      graphic_single_t graph;
      graph.header = header;
      graph.graphic_data_struct = va_arg(arg, graphic_data_t);
      length = sizeof(graphic_single_t);
      memcpy(data_buffer, &graph, length);
      break;
    }
    case 2: {
      header.data_cmd_id = UI_Data_ID_Draw2;
      graphic_double_t graph;
      graph.header = header;
      for (int i = 0; i < cnt; ++i) {
        graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
      }
      length = sizeof(graphic_double_t);
      memcpy(data_buffer, &graph, length);
      break;
    }
    case 5: {
      header.data_cmd_id = UI_Data_ID_Draw5;
      graphic_five_t graph;
      graph.header = header;
      for (int i = 0; i < cnt; ++i) {
        graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
      }
      length = sizeof(graphic_five_t);
      memcpy(data_buffer, &graph, length);
      break;
    }
    case 7: {
      header.data_cmd_id = UI_Data_ID_Draw7;
      graphic_seven_t graph;
      graph.header = header;
      for (int i = 0; i < cnt; ++i) {
        graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
      }
      length = sizeof(graphic_seven_t);
      memcpy(data_buffer, &graph, length);
      break;
    }
    default:;
  }
  return length;
}

int UserInterface::CharRefresh(uint8_t* data_buffer, graphic_data_t image, char* theString,
                               int len) {
  graphic_character_t char_graph;
  char_graph.header.data_cmd_id = UI_Data_ID_DrawChar;
  char_graph.header.sender_ID = Robot_ID_;
  char_graph.header.receiver_ID = Client_ID_;
  char_graph.graphic_data_struct = image;
  if (len > 30 || len <= 0) return -1;
  memset(char_graph.data, 0, sizeof(char_graph.data));
  memcpy(char_graph.data, theString, len);
  int length = sizeof(graphic_character_t);
  memcpy(data_buffer, &char_graph, length);
  return length;
}

void UserInterface::ChassisGUIInit(graphic_data_t *chassis, graphic_data_t *arrow, graphic_data_t *gimbal, graphic_data_t *empty1, graphic_data_t *empty2) {
    chassis_ = chassis;
    arrow_ = arrow;
    gimbal_ = gimbal;
    gimbalLen_ = 90;
    chassisLen_ = 90;
    LineDraw(chassis, "c", UI_Graph_Add, 1, UI_Color_Yellow, 60, chassisX_, chassisY_ - chassisLen_ / 2, chassisX_, chassisY_ + chassisLen_ / 2);
    LineDraw(arrow, "a", UI_Graph_Add, 1, UI_Color_Yellow, 20, chassisX_ - 7, chassisY_ + chassisLen_ / 2 - 7 , chassisX_ + 7, chassisY_ + chassisLen_ / 2 + 7);
    LineDraw(gimbal, "g", UI_Graph_Add, 0, UI_Color_White, 7, chassisX_, chassisY_, chassisX_, chassisY_ + gimbalLen_);
    LineDraw(empty1, "e1", UI_Graph_Add, 0, UI_Color_Yellow, 20, centerX_, centerY_ + 30, centerX_, centerY_ + 40);
    LineDraw(empty2, "e2", UI_Graph_Add, 0, UI_Color_Yellow, 20, centerX_, centerY_ + 50, centerX_, centerY_ + 60);
}

void UserInterface::ChassisGUIUpdate(float relative) {
    float x_end = chassisX_ + chassisLen_ / 2.0 * sinf(relative);
    float y_end = chassisY_ + chassisLen_ / 2.0 * cosf(relative);
    float x_start = chassisX_ - chassisLen_ / 2.0 * sinf(relative);
    float y_start = chassisY_ - chassisLen_ / 2.0 * cosf(relative);
    LineDraw(chassis_, "c", UI_Graph_Change, 1, UI_Color_Yellow, 60, (uint32_t)x_start, (uint32_t)y_start, (uint32_t)x_end, (uint32_t)y_end);
    LineDraw(arrow_, "a", UI_Graph_Change, 1, UI_Color_Yellow, 20, (uint32_t)(x_end - 10 * sinf(relative + M_PI/4)), (uint32_t)(y_end - 10 * cosf(relative + M_PI/4)),
             (uint32_t)(x_end + 10 * sinf(relative + M_PI/4)), (uint32_t)(y_end + 10 * cosf(relative + M_PI/4)));
    LineDraw(gimbal_, "g", UI_Graph_Change, 0, UI_Color_White, 7, chassisX_, chassisY_, chassisX_, chassisY_ + gimbalLen_);
}

void UserInterface::CrosshairGUI(graphic_data_t *crosshair1, graphic_data_t *crosshair2, graphic_data_t *crosshair3, graphic_data_t *crosshair4, graphic_data_t *crosshair5, graphic_data_t *crosshair6, graphic_data_t *crosshair7) {
    LineDraw(crosshair1, "ch1", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 50, centerY_ - 30, centerX_ + 50, centerY_ - 30);
    LineDraw(crosshair2, "ch2", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30, centerY_ - 40, centerX_ + 30, centerY_ - 40);
    LineDraw(crosshair3, "ch3", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30, centerY_ - 50, centerX_ + 30, centerY_ - 50);
    LineDraw(crosshair4, "ch4", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 50, centerY_ - 60, centerX_ + 50, centerY_ - 60);
    LineDraw(crosshair5, "ch5", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30, centerY_ - 70, centerX_ + 30, centerY_ - 70);
    LineDraw(crosshair6, "ch6", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30, centerY_ - 80, centerX_ + 30, centerY_ - 80);
    LineDraw(crosshair7, "ch7", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_, centerY_ - 30, centerX_, centerY_ - 100);
}

void UserInterface::CapGUIInit(graphic_data_t *barFrame, graphic_data_t *bar) {
    bar_ = bar;
    int x = barStartX_;
    int y = barStartY_;
    RectangleDraw(barFrame, "FM", UI_Graph_Add, 0, UI_Color_Yellow, 2, x, y, x + 310, y + 20);
    LineDraw(bar, "Bar", UI_Graph_Add, 0, UI_Color_Green, 10, x + 5, y + 10, x + 305, y + 10);
}

void UserInterface::CapGUIUpdate(float cap) {
    cap_ = cap;
    float offset = cap * 300;
    int x = barStartX_;
    int y = barStartY_;
    int color;
    uint32_t x_end = x + (uint32_t)offset;
    if (cap >= 0 && cap <= 0.3){
        color = UI_Color_Main;
    }
    else if (cap > 0.2 && cap < 0.95){
        color = UI_Color_Orange;
    }
    else color = UI_Color_Green;
    LineDraw(bar_, "Bar", UI_Graph_Change, 0, color, 10, x + 5, y + 10, x_end, y + 10);
}

void UserInterface::CapGUICharInit(graphic_data_t *percent) {
    percent_ = percent;
    percentLen_ = snprintf(percentStr_, 30, "%d%%", 100);
    CharDraw(percent, "PG", UI_Graph_Add, 2, UI_Color_Yellow, 15, percentLen_, 2, barStartX_ - 56, barStartY_ + 18);
}

void UserInterface::CapGUICharUpdate() {
    percentLen_ = snprintf(percentStr_, 30, "%d%%", (int)(cap_ * 100));
    CharDraw(percent_, "PG", UI_Graph_Change, 2, UI_Color_Yellow, 15, percentLen_, 2, barStartX_ - 56, barStartY_ + 18);
}

void UserInterface::DiagGUIInit(graphic_data_t *message, int len) {
    diag_ = message;
    CharDraw(message, "M0", UI_Graph_Add, 2, UI_Color_Main, 10, len, 2, diagStartX_, diagStartY_);
}

void UserInterface::DiagGUIUpdate(int len) {
    int currY = diagStartY_ - messageCount_ * 20;
    char name[10];
    snprintf(name, 10, "M%d", messageCount_);
    CharDraw(diag_, name, UI_Graph_Add, 2, UI_Color_Main, 10, len, 2, diagStartX_, currY);
}

void UserInterface::AddMessage(char *messageStr, int len, UserInterface *UI, Referee *referee, graphic_data_t *graph) {
    messageCount_++;
    if (messageCount_ > 15)
        return;
    UI->DiagGUIUpdate(len);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), *graph, messageStr, len);
}

void UserInterface::DiagGUIClear(UserInterface *UI, Referee *referee, graphic_data_t *graph, int currCount) {
    char str[] = " ";
    char name[10];
    snprintf(name, 10, "M%d", currCount);
    UI->CharDraw(diag_, name, UI_Graph_Change, 2, UI_Color_Main, 10, 30, 2, diagStartX_, diagStartY_ - currCount * 20);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), *graph, str, 1);
    referee->PrepareUIContent(communication::CHAR_GRAPH);

}

void UserInterface::ModeGUIInit(graphic_data_t *modeGraph, int len) {
    CharDraw(modeGraph, "MG", UI_Graph_Add, 0, UI_Color_Yellow, 15, 30, 2, chassisX_ - len * 15 / 2, modeStartY_);
}

void UserInterface::ModeGuiUpdate(graphic_data_t *modeGraph, int len) {
    CharDraw(modeGraph, "MG", UI_Graph_Change, 0, UI_Color_Yellow, 15, 30, 2, chassisX_ - len * 15 / 2, modeStartY_);
}

}  // namespace communication
