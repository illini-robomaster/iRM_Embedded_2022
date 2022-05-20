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

#include <cstring>

#include "crc_check.h"

static const uint8_t SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace communication {

bool Protocol::Receive(package_t package) {
  memcpy(bufferRx, package.data, package.length);
  int start_idx;
  int end_idx = 0;
  while (end_idx < package.length) {
    start_idx = end_idx;
    while (++end_idx < package.length && bufferRx[end_idx] != SOF)
      ;
    if (end_idx - start_idx > FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN) {
      int DATA_LENGTH = bufferRx[start_idx + 2] << BYTE | bufferRx[start_idx + 1];
      if (VerifyHeader(bufferRx + start_idx, FRAME_HEADER_LEN) &&
          VerifyFrame(bufferRx + start_idx,
                      FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN)) {
        int cmd_id = bufferRx[start_idx + FRAME_HEADER_LEN + 1] << BYTE |
                     bufferRx[start_idx + FRAME_HEADER_LEN];
        ProcessDataRx(cmd_id, bufferRx + start_idx + FRAME_HEADER_LEN + CMD_ID_LEN, DATA_LENGTH);
      }
    }
  }
  return true;
}

package_t Protocol::Transmit(int cmd_id) {
  bufferTx[0] = SOF;
  int DATA_LENGTH = ProcessDataTx(cmd_id, bufferTx + FRAME_HEADER_LEN + CMD_ID_LEN);
  if (DATA_LENGTH < 0) return package_t{nullptr, 0};
  bufferTx[1] = (uint8_t)((uint32_t)DATA_LENGTH & 0xFF);
  bufferTx[2] = (uint8_t)((uint32_t)DATA_LENGTH >> BYTE);
  AppendHeader(bufferTx, FRAME_HEADER_LEN);
  bufferTx[5] = (uint8_t)((uint32_t)cmd_id & 0xFF);
  bufferTx[6] = (uint8_t)((uint32_t)cmd_id >> BYTE);
  AppendFrame(bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN);
  return package_t{bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN};
}

bool Protocol::VerifyHeader(const uint8_t* data, int length) {
  return verify_crc8_check_sum(data, length);
}

bool Protocol::VerifyFrame(const uint8_t* data, int length) {
  return verify_crc16_check_sum(data, length);
}

void Protocol::AppendHeader(uint8_t* data, int length) { append_crc8_check_sum(data, length); }

void Protocol::AppendFrame(uint8_t* data, int length) { append_crc16_check_sum(data, length); }

bool Referee::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
  switch (cmd_id) {
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
    case POWER_HEAT_DATA:
      memcpy(&power_heat_data, data, length);
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
    case SHOOT_DATA:
      memcpy(&shoot_data, data, length);
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

int Referee::ProcessDataTx(int cmd_id, uint8_t* data) {
  UNUSED(cmd_id);
  UNUSED(data);

  /* TODO(neo): if we need to send message to referee in the future, please add handling code here
   */
  return -1;
}

bool Host::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
  switch (cmd_id) {
    case PACK:
      memcpy(&pack, data, length);
      break;
    case TARGET_ANGLE:
      memcpy(&target_angle, data, length);
      break;
    case NO_TARGET_FLAG:
      memcpy(&no_target_flag, data, length);
      break;
    case SHOOT_CMD:
      memcpy(&shoot_cmd, data, length);
      break;
    default:
      return false;
  }
  return true;
}

int Host::ProcessDataTx(int cmd_id, uint8_t* data) {
  int data_len;
  switch (cmd_id) {
    case PACK:
      data_len = sizeof(pack_t);
      memcpy(data, &pack, data_len);
      break;
    default:
      data_len = -1;
  }
  return data_len;
}

} /* namespace communication */
