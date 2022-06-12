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

#include "bsp_gpio.h"
#include "spi.h"

// acc (6 bytes) + temp (2 bytes) + gyro (6 bytes) + mag (6 bytes)
#define MPU6500_SIZEOF_DATA 20

// ist8310 error handling
#define IST8310_DATA_READY_BIT 2


namespace bsp {

typedef struct {
  float x;
  float y;
  float z;
} vec3f_t;

class MPU6500 : public GPIT {
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
  vec3f_t acce;
  // 3-axis gyroscope
  vec3f_t gyro;
  // 3-axis magnetometer
  vec3f_t mag;
  // sensor temperature
  float temp;
  // sensor timestamp
  uint32_t timestamp = 0;

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
  void IntCallback() override final;

  SPI_HandleTypeDef* hspi_;
  GPIO chip_select_;

  uint8_t io_buff_[MPU6500_SIZEOF_DATA + 1];  // spi tx+rx buffer

  // global interrupt wrapper
  // TODO(alvin): try to support multiple instances in the future
  static void SPITxRxCpltCallback(SPI_HandleTypeDef* hspi);
  static MPU6500* mpu6500;
};

class IST8310 : public GPIT {
 public:
  IST8310(I2C_HandleTypeDef* hi2c, uint16_t int_pin, const GPIO& reset);
  float mag[3];

 private:
  uint8_t Init();
  void IntCallback() override final;
  void ReadData(float mag_[3]);

  // polling interfaces used only in initialization
  uint8_t ReadReg(uint8_t reg);
  void WriteReg(uint8_t reg, uint8_t data);
  void ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);
  void WriteRegs(uint8_t reg, uint8_t *data, uint8_t len);

  I2C_HandleTypeDef *hi2c_;
  GPIO reset_;
};

typedef struct {
  uint8_t status;
  int16_t accel[3];
  int16_t temp;
  int16_t gyro[3];
} __packed BMI088_raw_data_t;

typedef struct {
  uint8_t status;
  float accel[3];
  float temp;
  float gyro[3];
  float time;
} BMI088_real_data_t;

enum {
  BMI088_NO_ERROR = 0x00,
  BMI088_ACC_PWR_CTRL_ERROR = 0x01,
  BMI088_ACC_PWR_CONF_ERROR = 0x02,
  BMI088_ACC_CONF_ERROR = 0x03,
  BMI088_ACC_SELF_TEST_ERROR = 0x04,
  BMI088_ACC_RANGE_ERROR = 0x05,
  BMI088_INT1_IO_CTRL_ERROR = 0x06,
  BMI088_INT_MAP_DATA_ERROR = 0x07,
  BMI088_GYRO_RANGE_ERROR = 0x08,
  BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
  BMI088_GYRO_LPM1_ERROR = 0x0A,
  BMI088_GYRO_CTRL_ERROR = 0x0B,
  BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
  BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

  BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
  BMI088_SELF_TEST_GYRO_ERROR = 0x40,
  BMI088_NO_SENSOR = 0xFF,
};

typedef struct {
  SPI_HandleTypeDef* hspi;
  GPIO_TypeDef* CS_ACCEL_Port;
  uint16_t CS_ACCEL_Pin;
  GPIO_TypeDef* CS_GYRO_Port;
  uint16_t CS_GYRO_Pin;
} BMI088_init_t;

class BMI088 {
 public:
  BMI088(BMI088_init_t init);
  BMI088(SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_ACCEL_Port, uint16_t CS_ACCEL_Pin, GPIO_TypeDef* CS_GYRO_Port, uint16_t CS_GYRO_Pin);
  bool IsReady();
  void Read(float gyro[3], float accel[3], float *temperate);
 private:
  SPI_HandleTypeDef* hspi_;
  GPIO_TypeDef* CS1_ACCEL_GPIO_Port_;
  uint16_t CS1_ACCEL_Pin_;
  GPIO_TypeDef* CS1_GYRO_GPIO_Port_;
  uint16_t CS1_GYRO_Pin_;

  uint8_t Init();

  bool bmi088_accel_init(void);
  bool bmi088_gyro_init(void);

  void BMI088_ACCEL_NS_L(void);
  void BMI088_ACCEL_NS_H(void);

  void BMI088_GYRO_NS_L(void);
  void BMI088_GYRO_NS_H(void);

  uint8_t BMI088_read_write_byte(uint8_t tx_data);

  void BMI088_write_single_reg(uint8_t reg, uint8_t data);
  void BMI088_read_single_reg(uint8_t reg, uint8_t *data);
  void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

  void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);
  void BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data);
  void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

  void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data);
  void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data);
  void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
};

} /* namespace bsp */
