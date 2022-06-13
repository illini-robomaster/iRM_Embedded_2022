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

#include "bsp_imu.h"

#include <cmath>

#include "bsp_error_handler.h"
#include "bsp_mpu6500_reg.h"
#include "bsp_ist8310_reg.h"
#include "bsp_bmi088_reg.h"
#include "bsp_os.h"

#define MPU6500_DELAY 55  // SPI delay
// configured with initialization sequences
#define MPU6500_ACC_FACTOR 4096.0f
#define MPU6500_GYRO_FACTOR 32.768f

#define GRAVITY_ACC 9.8f
#define DEG2RAD(x) ((x) / 180 * M_PI)

#define MAG_SEN 0.3f //raw int16 data change to uT unit. 原始整型数据变成 单位ut

#define IST8310_IIC_ADDRESS 0x0E  //the I2C address of IST8310

namespace bsp {

MPU6500* MPU6500::mpu6500 = nullptr;

MPU6500::MPU6500(SPI_HandleTypeDef* hspi, const GPIO& chip_select, uint16_t int_pin)
    : GPIT(int_pin), hspi_(hspi), chip_select_(chip_select) {
  const uint8_t init_len = 7;
  const uint8_t init_data[init_len][2] = {
      {MPU6500_PWR_MGMT_1, 0x03},      // auto select clock source
      {MPU6500_PWR_MGMT_2, 0x00},      // enable acc & gyro
      {MPU6500_CONFIG, 0x02},          // gyro LP bandwidth 92Hz
      {MPU6500_GYRO_CONFIG, 0x10},     // gyro range 1000 dps / 32.8
      {MPU6500_ACCEL_CONFIG, 0x10},    // acc range 8g / 4096
      {MPU6500_ACCEL_CONFIG_2, 0x02},  // acc LP bandwidth 92Hz
      {MPU6500_INT_PIN_CFG, 0x10},     // any read to clear interrupt
  };
  Reset();  // reset all registers and signal paths
  for (size_t i = 0; i < init_len; ++i) WriteReg(init_data[i][0], init_data[i][1]);
  // validate register values
  uint8_t tmp;
  for (size_t i = 0; i < init_len; ++i) {
    ReadReg(init_data[i][0], &tmp);
    if (tmp != init_data[i][1])
      bsp_error_handler(__FUNCTION__, __LINE__, "imu register incorrect initialization");
  }
  // setup interrupt callback
  RM_ASSERT_FALSE(mpu6500, "Repteated intialization of MPU6500");
  mpu6500 = this;
  HAL_SPI_RegisterCallback(hspi, HAL_SPI_TX_RX_COMPLETE_CB_ID, &MPU6500::SPITxRxCpltCallback);
  // initialize magnetometer
  IST8310Init();
  // enable imu interrupt
  WriteReg(MPU6500_INT_ENABLE, 0x01);
}

void MPU6500::IST8310Init() {
  WriteReg(MPU6500_USER_CTRL, 0x30);     // enable I2C master and reset all slaves
  WriteReg(MPU6500_I2C_MST_CTRL, 0x0d);  // 400 kHz I2C clock
  // slave 0 for auto receive
  WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0e | 0x80);  // read from device 0x0e
  WriteReg(MPU6500_I2C_SLV0_REG, 0x03);          // read data from 0x03 reg
  // slave 1 for auto transmit
  WriteReg(MPU6500_I2C_SLV1_ADDR, 0x0e);  // write into device 0x0e
  WriteReg(MPU6500_I2C_SLV1_REG, 0x0a);   // write data into 0x0a reg
  WriteReg(MPU6500_I2C_SLV1_DO, 0x01);    // send measurement command
  // enable slave 0 and 1
  WriteReg(MPU6500_I2C_SLV0_CTRL, 0xd6);  // swap endian + 6 bytes rx
  WriteReg(MPU6500_I2C_SLV1_CTRL, 0x81);  // 1 bytes tx
}

void MPU6500::UpdateData() {
  chip_select_.Low();
  io_buff_[0] = MPU6500_ACCEL_XOUT_H | 0x80;
  HAL_SPI_TransmitReceive_DMA(hspi_, io_buff_, io_buff_, MPU6500_SIZEOF_DATA + 1);
}

void MPU6500::Reset() {
  WriteReg(MPU6500_PWR_MGMT_1, 0x80);
  WriteReg(MPU6500_SIGNAL_PATH_RESET, 0x07);
  WriteReg(MPU6500_USER_CTRL, 0x03);

  HAL_Delay(1);  // seems like signal path reset needs some time
}

void MPU6500::WriteReg(uint8_t reg, uint8_t data) { WriteRegs(reg, &data, 1); }

void MPU6500::WriteRegs(uint8_t reg_start, uint8_t* data, uint8_t len) {
  uint8_t tx = reg_start & 0x7f;

  chip_select_.Low();
  HAL_SPI_Transmit(hspi_, &tx, 1, MPU6500_DELAY);
  HAL_SPI_Transmit(hspi_, data, len, MPU6500_DELAY);
  chip_select_.High();
}

void MPU6500::ReadReg(uint8_t reg, uint8_t* data) { ReadRegs(reg, data, 1); }

void MPU6500::ReadRegs(uint8_t reg_start, uint8_t* data, uint8_t len) {
  chip_select_.Low();
  *data = static_cast<uint8_t>(reg_start | 0x80);
  HAL_SPI_Transmit(hspi_, data, 1, MPU6500_DELAY);
  HAL_SPI_Receive(hspi_, data, len, MPU6500_DELAY);
  chip_select_.High();
}

void MPU6500::SPITxRxCpltCallback() {
  chip_select_.High();
  // NOTE(alvin): per MPU6500 documentation, the first byte of the rx / tx buffer
  //              contains the address of the SPI device
  uint8_t* buff = io_buff_ + 1;
  int16_t* array = reinterpret_cast<int16_t*>(buff);
  // in-place swap endian
  for (size_t i = 0; i < MPU6500_SIZEOF_DATA; i += 2)
    array[i / 2] = (int16_t)(buff[i] << 8 | buff[i + 1]);

  acce.x() = (float)array[0] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
  acce.y() = (float)array[1] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
  acce.z() = (float)array[2] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
  temp = (float)array[3] / MPU6500_TEMP_FACTOR + MPU6500_TEMP_OFFSET;
  gyro.x() = DEG2RAD((float)array[4] / MPU6500_GYRO_FACTOR);
  gyro.y() = DEG2RAD((float)array[5] / MPU6500_GYRO_FACTOR);
  gyro.z() = DEG2RAD((float)array[6] / MPU6500_GYRO_FACTOR);
  mag.x() = (float)array[7];
  mag.y() = (float)array[8];
  mag.z() = (float)array[9];
}

void MPU6500::IntCallback() {
  timestamp = GetHighresTickMicroSec();
  UpdateData();
}

void MPU6500::SPITxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  UNUSED(hspi);
  mpu6500->SPITxRxCpltCallback();
}

IST8310* IST8310::ist8310 = nullptr;

void I2CRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  UNUSED(hi2c);

  // copy magnetic data from raw buffer
  IST8310* sensor = IST8310::ist8310;
  sensor->mag[0] = MAG_SEN * (int16_t)((sensor->buf_[1] << 8) | sensor->buf_[0]);
  sensor->mag[1] = MAG_SEN * (int16_t)((sensor->buf_[3] << 8) | sensor->buf_[2]);
  sensor->mag[2] = MAG_SEN * (int16_t)((sensor->buf_[5] << 8) | sensor->buf_[4]);

  // call user callback
  if (IST8310::ist8310->callback_) {
    IST8310::ist8310->callback_(*IST8310::ist8310);
  }
}

IST8310::IST8310(I2C_HandleTypeDef* hi2c, uint16_t int_pin, const GPIO& reset)
    : GPIT(int_pin),
      hi2c_(hi2c),
      reset_(reset) {

  // set global pointer
  RM_ASSERT_EQ(ist8310, nullptr, "Repeated initialization of IST8310");
  ist8310 = this;

  // register I2C rx complete callback
  HAL_I2C_RegisterCallback(hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, I2CRxCpltCallback);

  // reset device
  const uint8_t wait_time = 1;
  const uint8_t sleepTime = 50;
  // reset is active low
  reset_.Low();
  HAL_Delay(sleepTime);
  reset_.High();
  HAL_Delay(sleepTime);

  if (ReadReg(IST8310_WHO_AM_I) != IST8310_WHO_AM_I_VALUE) {
    bsp_error_handler(__FUNCTION__, __LINE__, "No IST8310 sensor");
  }

  // init sequence
  const uint32_t init_len = 4;
  // the first column:the registers of IST8310.
  // the second column: the value to be writed to the registers.
  const uint8_t init_data[init_len][2] = {
      {IST8310_CNTL2, IST8310_DREN},  // enable interrupt and low pin polarity.
      {IST8310_AVGCNTL, IST8310_Y_AVG_2 | IST8310_XZ_AVG_2}, // average 2 times.
      {IST8310_PDCNTL, IST8310_PD_NORMAL}, // normal pulse duration (required by documentation)
      {IST8310_CNTL1, 0x0B},     //200Hz output rate TODO(alvin): this is undocumented.
  };

  //set sensor config and check for errors
  for (uint8_t i = 0; i < init_len; ++i) {
    WriteReg(init_data[i][0], init_data[i][1]);
    HAL_Delay(wait_time);
    if (ReadReg(init_data[i][0]) != init_data[i][1]) {
      bsp_error_handler(__FUNCTION__, __LINE__, "IST8310 init failed");
    }
    HAL_Delay(wait_time);
  }
}

void IST8310::RegisterCallback(IST8310_Callback callback) {
  callback_ = callback;
}

void IST8310::IntCallback() {
  // set data ready timestamp
  timestamp = GetHighresTickMicroSec();

  // read data register async
  HAL_I2C_Mem_Read_DMA(hi2c_, IST8310_IIC_ADDRESS << 1, IST8310_DATA,
                       I2C_MEMADD_SIZE_8BIT, buf_, 6);

  if (callback_) { callback_(*this); }
}

uint8_t IST8310::ReadReg(uint8_t reg) {
  uint8_t res = 0;
  ReadRegs(reg, &res, 1);
  return res;
}

void IST8310::WriteReg(uint8_t reg, uint8_t data) {
  WriteRegs(reg, &data, 1);
}

void IST8310::ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  const uint32_t timeout = 10;
  HAL_I2C_Mem_Read(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,buf,len,timeout);
}

void IST8310::WriteRegs(uint8_t reg, uint8_t* data, uint8_t len) {
  const uint32_t timeout = 10;
  HAL_I2C_Mem_Write(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,data,len,timeout);
}

BMI088::BMI088(BMI088_init_t init) {
  hspi_ = init.hspi;
  CS1_ACCEL_GPIO_Port_ = init.CS_ACCEL_Port;
  CS1_ACCEL_Pin_ = init.CS_ACCEL_Pin;
  CS1_GYRO_GPIO_Port_ = init.CS_GYRO_Port;
  CS1_GYRO_Pin_ = init.CS_GYRO_Pin;
  Init();
}

BMI088::BMI088(SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_ACCEL_Port, uint16_t CS_ACCEL_Pin, GPIO_TypeDef* CS_GYRO_Port, uint16_t CS_GYRO_Pin) {
  hspi_ = hspi;
  CS1_ACCEL_GPIO_Port_ = CS_ACCEL_Port;
  CS1_ACCEL_Pin_ = CS_ACCEL_Pin;
  CS1_GYRO_GPIO_Port_ = CS_GYRO_Port;
  CS1_GYRO_Pin_ = CS_GYRO_Pin;
  Init();
}

bool BMI088::IsReady() {
  return HAL_SPI_Init(hspi_) == HAL_OK;
}

void BMI088::BMI088_ACCEL_NS_L() {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port_, CS1_ACCEL_Pin_, GPIO_PIN_RESET);
}

void BMI088::BMI088_ACCEL_NS_H() {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port_, CS1_ACCEL_Pin_, GPIO_PIN_SET);
}

void BMI088::BMI088_GYRO_NS_L() {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port_, CS1_GYRO_Pin_, GPIO_PIN_RESET);
}

void BMI088::BMI088_GYRO_NS_H() {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port_, CS1_GYRO_Pin_, GPIO_PIN_SET);
}

uint8_t BMI088::BMI088_read_write_byte(uint8_t tx_data) {
  uint8_t rx_data;
  HAL_SPI_TransmitReceive(hspi_, &tx_data, &rx_data, 1, 1000);
  return rx_data;
}

void BMI088::BMI088_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_read_write_byte(reg);
  BMI088_read_write_byte(data);
}

void BMI088::BMI088_read_single_reg(uint8_t reg, uint8_t* data) {
  BMI088_read_write_byte(reg | 0x80);
  *data = BMI088_read_write_byte(0x55);
}

void BMI088::BMI088_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
  BMI088_read_write_byte(reg | 0x80);
  while (len != 0) {
    *buf = BMI088_read_write_byte(0x55);
    buf++;
    len--;
  }
}

void BMI088::BMI088_accel_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_ACCEL_NS_L();
  BMI088_write_single_reg(reg, data);
  BMI088_ACCEL_NS_H();
}

void BMI088::BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data) {
  BMI088_ACCEL_NS_L();
  BMI088_read_write_byte(reg | 0x80);
  BMI088_read_write_byte(0x55);
  *data = BMI088_read_write_byte(0x55);
  BMI088_ACCEL_NS_H();
}

void BMI088::BMI088_accel_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
  BMI088_ACCEL_NS_L();
  BMI088_read_write_byte(reg | 0x80);
  BMI088_read_muli_reg(reg, buf, len);
  BMI088_ACCEL_NS_H();
}

void BMI088::BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_GYRO_NS_L();
  BMI088_write_single_reg(reg, data);
  BMI088_GYRO_NS_H();
}

void BMI088::BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data) {
  BMI088_GYRO_NS_L();
  BMI088_read_single_reg(reg, data);
  BMI088_GYRO_NS_H();
}

void BMI088::BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
  BMI088_GYRO_NS_L();
  BMI088_read_muli_reg(reg, buf, len);
  BMI088_GYRO_NS_H();
}

static float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

uint8_t BMI088::Init() {
  uint8_t error = BMI088_NO_ERROR;

  error |= bmi088_accel_init();
  error |= bmi088_gyro_init();

  hspi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  return error;
}

bool BMI088::bmi088_accel_init() {
  uint8_t res = 0;
  uint8_t write_reg_num;

  //check commiunication
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);

  //accel software reset
  BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  HAL_Delay(BMI088_LONG_DELAY_TIME);

  //check commiunication is normal after reset
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);

  // check the "who am I"
  if (res != BMI088_ACC_CHIP_ID_VALUE)
    return false;

  //set accel sonsor config and check
  for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; ++write_reg_num) {
    BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], &res);
    HAL_Delay(1);
    if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
      return write_BMI088_accel_reg_data_error[write_reg_num][2];
  }
  return false;
}

bool BMI088::bmi088_gyro_init() {
  uint8_t write_reg_num;
  uint8_t res = 0;

  //check commiunication
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);

  //reset the gyro sensor
  BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  HAL_Delay(BMI088_LONG_DELAY_TIME);
  //check commiunication is normal after reset
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);

  // check the "who am I"
  if (res != BMI088_GYRO_CHIP_ID_VALUE)
    return false;

  //set gyro sonsor config and check
  for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; ++write_reg_num) {
    BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], &res);
    HAL_Delay(1);
    if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
      return write_BMI088_gyro_reg_data_error[write_reg_num][2];
  }

  return false;
}

void BMI088::Read(float* gyro, float* accel, float* temperate) {
  uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
  int16_t bmi088_raw_temp;

  BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

  bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
  accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
  bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
  accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
  bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
  accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

  BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
  if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
  }
  BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

  bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

  if (bmi088_raw_temp > 1023)
    bmi088_raw_temp -= 2048;

  *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

} /* namespace bsp */
