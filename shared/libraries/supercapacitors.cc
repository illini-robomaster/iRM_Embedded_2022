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

#include "supercapacitors.h"

namespace power {

supercapacitors::supercapacitors(bsp::CAN* can) {
  can_ = can;
}

void supercapacitors::UpdatePowerLimit(float power) {
  uint8_t data[8] = {0};
  uint16_t power_send = (int)(power * 100);
  data[0] = power_send >> 8;
  data[1] = power_send;
  can_->Transmit(sc_addr, data, 8);
}

}
