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

static const uint8_t SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace RoboMaster {
    bool Referee::Update(const uint8_t* data, int length) {
      memcpy(buffer, data, length);
      int start_idx;
      int end_idx = 0;
      while (end_idx < length) {
        start_idx = end_idx;
        for (; ++end_idx < length && buffer[end_idx] != SOF;);
        if (end_idx - start_idx > FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN) {
          int DATA_LENGTH = buffer[start_idx + 2] << BYTE | buffer[start_idx + 1];
          if (CheckHeader(buffer + start_idx, FRAME_HEADER_LEN) &&
              CheckFrame(buffer + start_idx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN)) {
            int cmd_id = buffer[start_idx + FRAME_HEADER_LEN + 1] << BYTE | buffer[start_idx + FRAME_HEADER_LEN];
            ProcessData(cmd_id , buffer + start_idx + FRAME_HEADER_LEN + CMD_ID_LEN, DATA_LENGTH);
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
        case SHOOT_DATA:
          memcpy(&shoot_data, data, length);
          break;
        case GAME_STATUS:
          memcpy(&game_status, data, length);
          break;
        case GAME_RESULT:
          memcpy(&game_result, data, length);
          break;
        case GAME_ROBOT_HP:
          memcpy(&game_robot_HP, data, length);
          break;
        case EVENT_DATA:
          memcpy(&event_data, data, length);
          break;          
        case SUPPLY_PROJECTILE_ACTION:
          memcpy(&supply_projectile_action, data, length);
          break;
        case REFEREE_WARNING:
          memcpy(&referee_warning, data, length);
          break;  
        case DART_REMAINING_TIME:
          memcpy(&dart_remaining_time, data, length);
          break;  
        case GAME_ROBOT_STATUS:
          memcpy(&game_robot_status, data, length);
          break;
        case GAME_ROBOT_POS:
          memcpy(&game_robot_pos, data, length);
          break;
        case BUFF:
          memcpy(&buff, data, length);
          break;
        case AERIAL_ROBOT_ENERGY:
          memcpy(&aerial_robot_energy, data, length);
          break;
        case ROBOT_HURT:
          memcpy(&robot_hurt, data, length);
          break;
        case BULLET_REMAINING:
          memcpy(&bullet_remaining, data, length);
          break;  
        case RFID_STATUS:
          memcpy(&rfid_status, data, length);
          break; 
        case DART_CLIENT_CMD:
          memcpy(&dart_client_cmd, data, length);
          break; 
        default:
          return false;
      }
      return true;
    }
}
