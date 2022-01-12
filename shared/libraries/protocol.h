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

namespace communication {

constexpr int MAX_FRAME_LEN = 300;

typedef struct {
  uint8_t* data,
  int length,
} package_t;

class protocol {
public:
  /**
     * @brief update the information from referee system
     *
     * @param data      address for received data read from UART for referee
     * @param length    number of bytes in received data
     * @return true for success; false for failure
   */
  bool Receive(package_t package);

  virtual package_t Transmit(int cmd_id) = 0;

private:
  uint8_t bufferRx[MAX_FRAME_LEN];
  uint8_t bufferTx[MAX_FRAME_LEN];
  /**
     * @brief verify the header of frame with crc8
     *
     * @param data      address for header data
     * @param length    number of bytes in header data
     * @return true for success; false for failure
   */
  bool VerifyHeader(const uint8_t* data, int length);
  /**
     * @brief verify the frame with crc16
     *
     * @param data      address for frame data`
     * @param length    number of bytes in frame data
     * @return true for success; false for failure
   */
  bool VerifyFrame(const uint8_t* data, int length);
  /**
     * @brief process the data for certain command and update corresponding status variables
     *
     * @param cmd_id    command id
     * @param data      address for command data
     * @param length    number of bytes in command data
     * @return true for success; false for failure
   */
  virtual bool ProcessData(int cmd_id, const uint8_t* data, int length) = 0;

  /**
   * @brief append the header of frame with crc8
   *
   * @param data        address for header data
   * @param length      number of bytes in header data
   * @return 0 for fail, 1 for success
   */
  bool AppendHeader(uint8_t* data, int length);

  /**
   * @brief append the frame with crc16
   *
   * @param data        address for frame data
   * @param length      number of bytes in frame data
   * @return 0 for fail, 1 for success
   */
  bool AppendFrame(uint8_t* data, int length);
};

class host: public protocol  {

};

class referee: public protocol {

};

}
