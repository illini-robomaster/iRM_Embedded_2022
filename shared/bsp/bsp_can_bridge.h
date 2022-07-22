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

#pragma once

#include "bsp_can.h"

namespace bsp {

typedef enum {
  VX,
  VY,
  RELATIVE_ANGLE,
  START,
  MODE,
  DEAD,
  SHOOTER_POWER,
  COOLING_HEAT,
  COOLING_LIMIT,
  SPEED_LIMIT,
} can_bridge_cmd;

typedef struct {
  uint8_t id;
  union {
    float data_float;
    int data_int;
    bool data_bool;
  };
} bridge_data_t;

class CanBridge {
 public:
  CanBridge(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);
  void UpdateData(const uint8_t data[]);
  void TransmitOutput();

  bridge_data_t cmd;
  float vx = 0;
  float vy = 0;
  float relative_angle = 0;
  bool start = false;
  int mode = 0;
  bool dead = false;
  float shooter_power = 0;
  float cooling_heat = 0;
  float cooling_limit = 0;
  float speed_limit = 0;

 private:
  bsp::CAN* can_;
  uint16_t rx_id_;
  uint16_t tx_id_;
};

}  // namespace bsp
