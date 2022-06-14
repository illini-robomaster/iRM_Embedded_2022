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

#include <stddef.h>

#include <Eigen/Dense>

#include "bsp_gpio.h"
#include "i2c.h"
#include "spi.h"

// acc (6 bytes) + temp (2 bytes) + gyro (6 bytes) + mag (6 bytes)
#define MPU6500_SIZEOF_DATA 20

#define BMI088_TX_SIZE 1  // 1 byte for tx data

// 0x02 - 0x07
#define BMI088_GYRO_SIZE 6
// 0x12 - 0x23 + 1 extra byte for dummy data (see doc)
#define BMI088_ACCEL_SIZE 19

#define BMI088_GYRO_BUF_SIZE (BMI088_GYRO_SIZE + BMI088_TX_SIZE)
#define BMI088_ACCEL_BUF_SIZE (BMI088_ACCEL_SIZE + BMI088_TX_SIZE)
#define BMI088_TEMP_BUF_OFFSET (BMI088_TEMP_M - BMI088_ACCEL_XOUT_L)

namespace bsp {

class MPU6500 {
 public:
  /**
   * @brief constructor for a MPU6500 IMU sensor
   *
   * @param hspi         HAL SPI handle associated with the sensor
   * @param chip_select  chip select gpio pin
   * @param int_pin      interrupt pin number
   */
  MPU6500(SPI_HandleTypeDef* hspi, const GPIO& chip_select, uint16_t int_pin);

  /**
   * @brief reset sensor registers
   */
  void Reset();

  // 3-axis accelerometer
  Eigen::Vector3f acce;
  // 3-axis gyroscope
  Eigen::Vector3f gyro;
  // 3-axis magnetometer
  Eigen::Vector3f mag;
  // sensor temperature
  float temp;
  // sensor timestamp
  uint32_t timestamp = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  /**
   * @brief sample latest sensor data
   */
  void UpdateData();

  void IST8310Init();
  void WriteReg(uint8_t reg, uint8_t data);
  void WriteRegs(uint8_t reg_start, uint8_t* data, uint8_t len);
  void ReadReg(uint8_t reg, uint8_t* data);
  void ReadRegs(uint8_t reg_start, uint8_t* data, uint8_t len);

  void SPITxRxCpltCallback();

  SPI_HandleTypeDef* hspi_;
  GPIT int_;
  GPIO chip_select_;

  uint8_t io_buff_[MPU6500_SIZEOF_DATA + 1];  // spi tx+rx buffer

  // global interrupt wrapper
  // NOTE(alvin): currently support single instance only
  friend void MPU6500IntCallback(void* data);
  static void SPITxRxCpltCallback(SPI_HandleTypeDef* hspi);
  static MPU6500* mpu6500;
};

class IST8310 {
  typedef void (*IST8310_Callback)(const IST8310& sensor);

 public:
  IST8310(I2C_HandleTypeDef* hi2c, uint16_t int_pin, const GPIO& reset);
  void RegisterCallback(IST8310_Callback callback);

  Eigen::Vector3f mag;
  uint32_t timestamp = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  uint8_t Init();

  // polling interfaces used only in initialization
  uint8_t ReadReg(uint8_t reg);
  void WriteReg(uint8_t reg, uint8_t data);
  void ReadRegs(uint8_t reg, uint8_t* data, uint8_t len);
  void WriteRegs(uint8_t reg, uint8_t* data, uint8_t len);
  void IntCallback();

  friend void IST8310IntCallback(void* data);
  friend void IST8310I2cRxCpltCallback(I2C_HandleTypeDef* hi2c);
  static IST8310* ist8310;

  IST8310_Callback callback_ = nullptr;
  uint8_t buf_[6];

  I2C_HandleTypeDef* hi2c_;
  GPIT int_;
  GPIO reset_;
};

class BMI088 {
  typedef void (*CallbackTypeDef)(const BMI088&);

 public:
  BMI088(SPI_HandleTypeDef* hspi, const GPIO& accel_cs, const GPIO& gyro_cs, uint16_t accel_int_pin,
         uint16_t gyro_int_pin);

  void Read(float gyro[3], float accel[3], float* temperate);

  void RegisterAccelCallback(CallbackTypeDef callback);
  void RegisterGyroCallback(CallbackTypeDef callback);

  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  float temperature;

  uint32_t accel_timestamp = 0;
  uint32_t gyro_timestamp = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  SPI_HandleTypeDef* hspi_;
  // chip select pins
  GPIO accel_cs_;
  GPIO gyro_cs_;
  // interrupt pins
  GPIT accel_int_;
  GPIT gyro_int_;

  CallbackTypeDef accel_callback_ = nullptr;
  CallbackTypeDef gyro_callback_ = nullptr;

  bool spi_busy_ = false;
  bool accel_pending_ = false;
  bool gyro_pending_ = false;

  uint8_t accel_buf_[BMI088_ACCEL_BUF_SIZE];
  uint8_t gyro_buf_[BMI088_GYRO_BUF_SIZE];

  // polling interfaces used only in initialization
  uint8_t AccelReadReg(uint8_t reg);
  void AccelReadRegs(uint8_t reg, uint8_t* data, uint8_t len);
  void AccelWriteReg(uint8_t reg, uint8_t data);
  void AccelWriteRegs(uint8_t reg, uint8_t* data, uint8_t len);

  uint8_t GyroReadReg(uint8_t reg);
  void GyroReadRegs(uint8_t reg, uint8_t* data, uint8_t len);
  void GyroWriteReg(uint8_t reg, uint8_t data);
  void GyroWriteRegs(uint8_t reg, uint8_t* data, uint8_t len);

  void AccelInit();
  void AccelRead();
  void AccelIntCallback();

  void GyroInit();
  void GyroRead();
  void GyroIntCallback();

  friend void BMI088AccelIntCallback(void* data);
  friend void BMI088GyroIntCallback(void* data);
  friend void BMI088SpiTxRxCpltCallback(SPI_HandleTypeDef* hspi);
};

} /* namespace bsp */
