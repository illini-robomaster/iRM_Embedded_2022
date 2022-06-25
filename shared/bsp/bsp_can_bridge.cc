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

namespace bsp {

static void bridge_callback(const uint8_t data[], void* args) {
  CanBridge* bridge = reinterpret_cast<CanBridge*>(args);
  bridge->UpdateData(data);
}

CanBridge::CanBridge(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id) {
  can_ = can;
  rx_id_ = rx_id;
  tx_id_ = tx_id;
  can_->RegisterRxCallback(rx_id_, bridge_callback, this);
}

void CanBridge::UpdateData(const uint8_t* data) {
  for (int i = 0; i < MAX_IO; ++i) IO[i] = data[i];
}

void CanBridge::TransmitOutput(uint8_t* IO_data) { can_->Transmit(tx_id_, IO_data, MAX_IO); }

}  // namespace bsp
