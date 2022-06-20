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

#include "lidar07.h"

namespace distance {

LIDAR07::LIDAR07(UART_HandleTypeDef* huart, lidar07_delay_t delay_func) : bsp::UART(huart) {
  SetupRx(100);
  SetupTx(100);
  delay_func_ = delay_func;
}

bool LIDAR07::begin() {
  readVersionPacket.head = 0xF5;
  readVersionPacket.command = 0x43;
  readVersionPacket.data[0] = 0x00;
  readVersionPacket.data[1] = 0x00;
  readVersionPacket.data[2] = 0x00;
  readVersionPacket.data[3] = 0x00;
  readVersionPacket.checkData[0] = 0xAC;
  readVersionPacket.checkData[1] = 0x45;
  readVersionPacket.checkData[2] = 0x62;
  readVersionPacket.checkData[3] = 0x3B;

  setModePacket.head = 0xF5;
  setModePacket.command = 0xE1;
  setModePacket.data[0] = 0x00;
  setModePacket.data[1] = 0x00;
  setModePacket.data[2] = 0x00;
  setModePacket.data[3] = 0x00;
  setModePacket.checkData[0] = 0xA5;
  setModePacket.checkData[1] = 0x8D;
  setModePacket.checkData[2] = 0x89;
  setModePacket.checkData[3] = 0xA7;

  startPacket.head = 0xF5;
  startPacket.command = 0xE0;
  startPacket.data[0] = 0x01;
  startPacket.data[1] = 0x00;
  startPacket.data[2] = 0x00;
  startPacket.data[3] = 0x00;
  startPacket.checkData[0] = 0x9F;
  startPacket.checkData[1] = 0x70;
  startPacket.checkData[2] = 0xE9;
  startPacket.checkData[3] = 0x32;

  stopPacket.head = 0xF5;
  stopPacket.command = 0xE0;
  stopPacket.data[0] = 0x00;
  stopPacket.data[1] = 0x00;
  stopPacket.data[2] = 0x00;
  stopPacket.data[3] = 0x00;
  stopPacket.checkData[0] = 0x28;
  stopPacket.checkData[1] = 0xEA;
  stopPacket.checkData[2] = 0x84;
  stopPacket.checkData[3] = 0xEE;

  startFilterPacket.head = 0xF5;
  startFilterPacket.command = 0xD9;
  startFilterPacket.data[0] = 0x01;
  startFilterPacket.data[1] = 0x00;
  startFilterPacket.data[2] = 0x00;
  startFilterPacket.data[3] = 0x00;
  startFilterPacket.checkData[0] = 0xB7;
  startFilterPacket.checkData[1] = 0x1F;
  startFilterPacket.checkData[2] = 0xBA;
  startFilterPacket.checkData[3] = 0xBA;

  stopFilterPacket.head = 0xF5;
  stopFilterPacket.command = 0xD9;
  stopFilterPacket.data[0] = 0x00;
  stopFilterPacket.data[1] = 0x00;
  stopFilterPacket.data[2] = 0x00;
  stopFilterPacket.data[3] = 0x00;
  stopFilterPacket.checkData[0] = 0x00;
  stopFilterPacket.checkData[1] = 0x85;
  stopFilterPacket.checkData[2] = 0xD7;
  stopFilterPacket.checkData[3] = 0x66;

  bool ret = false;
  uint8_t* buff;

  delay_func_(500);

  Write((uint8_t*)&readVersionPacket, sizeof(SendPacket_t));
  delay_func_(100);
  Read(&buff);

  if (crc32Check(buff, 8)) {
    readValue(buff, LIDAR07_VERSION);
    ret = true;
  }
  return setMeasureMode() && ret;
}

bool LIDAR07::setMeasureMode() {
  uint8_t* buff;

  delay_func_(20);

  Write((uint8_t*)&setModePacket, sizeof(SendPacket_t));
  delay_func_(100);
  Read(&buff);

  return crc32Check(buff, 8);
}

bool LIDAR07::crc32Check(uint8_t* buff, uint8_t len) {
  uint8_t data;
  uint8_t ret  = false;
  uint32_t checkData;
  uint32_t crc = 0xFFFFFFFF;          //Original value
  uint8_t buffCheck[len];

  checkData = (uint32_t)buff[len] | ((uint32_t)buff[len+1]<<8) | ((uint32_t)buff[len+2]<<16) | ((uint32_t)buff[len+3]<<24);
  for(uint8_t i=0;i<len;i++)
    buffCheck[i] = buff[i];
  for (uint8_t i=0; i<len; i++){
    data = buffCheck[i];
    crc = crc ^ ((uint32_t)data<<24);        //8 bits XOR higher than the crc original value
    for (uint8_t y = 0; y < 8; y++){         //Cycle 8 bits
      if (crc & 0x80000000)                  //The bit shifted to the left is 1, and after shifting to the left, it is XORed with the polynomial
        crc = (crc << 1) ^0x04C11DB7;
      else                                   //Or shift to left directly
        crc <<= 1;
    }
  }
  crc = crc^0x00;                             //At last, return to XOR with the XOR value of result
  if(crc == checkData)
    ret =true;
  return ret;                                //Return the final check value
}

void LIDAR07::readValue(uint8_t* buff, uint8_t type) {
  if(type == LIDAR07_VERSION){
    version = (uint32_t)buff[4] | ((uint32_t)buff[5]<<8) | ((uint32_t)buff[6]<<16) | ((uint32_t)buff[7]<<24);
  } else if(type == LIDAR07_MEASURE){
    distance = (uint16_t)buff[4] | ((uint16_t)buff[5]<<8);
    amplitude = (uint16_t)buff[8] | ((uint16_t)buff[9]<<8);
  }
}

}
