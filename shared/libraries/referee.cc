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

#include "referee.h"
#include "crc_check.h"

#include <cstring>

static const int SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace RoboMaster {
    bool Referee::Update(const uint8_t* data, int length) {
      uint8_t debug[300];
      memcpy(debug, data, length);
      uint8_t gg = data[0];
      int start_idx;
      int end_idx = 0;
      while (end_idx < length) {
        start_idx = end_idx;
        for (; end_idx < length; ++end_idx) {
          uint8_t tmp = data[0];
          if (tmp == SOF) {
            //set
            start_idx = gg;
          }
        }
        for (; ++end_idx < length && data[end_idx] != SOF;);
        if (end_idx - start_idx > FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN) {
          int DATA_LENGTH = data[start_idx + 2] << BYTE | data[start_idx + 1];
          if (CheckHeader(data + start_idx, FRAME_HEADER_LEN) && CheckFrame(data + start_idx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN)) {
            int cmd_id = data[start_idx + FRAME_HEADER_LEN + 1] | data[start_idx + FRAME_HEADER_LEN];
            if (!ProcessData(cmd_id , data + start_idx + FRAME_HEADER_LEN + CMD_ID_LEN, DATA_LENGTH))
              return false;
          }
        }
      }
      return true;
    }

    bool Referee::CheckHeader(const uint8_t* data, int length) {
      return verify_crc8_check_sum(data, length);
    }

    bool Referee::CheckFrame(const uint8_t* data, int length) {
      return verify_crc16_check_sum(data, length);
    }

    bool Referee::ProcessData(int cmd_id, const uint8_t *data, int length) {
      switch (cmd_id) {
        case POWER_HEAT_DATA:
          memcpy(&power_heat_data, data, length);
          break;
        default:
          return false;
      }
      return true;
    }
}
