

#include "bsp_imu_i2c.h"
#include "utils.h"

namespace bsp {

WT901::WT901(I2C_HandleTypeDef* i2c, uint16_t DevAddr) {
  i2c_ = i2c;
  DevAddr_ = DevAddr << 1;
  for (int i = 0; i < 3; i++)
    angle_offset_[i] = 0;
}

bool WT901::IsReady() {
  return HAL_I2C_IsDeviceReady(i2c_, DevAddr_, 1, 100) == HAL_OK;
}

bool WT901::GetAngle(float *angle) {
  if (!GetRawAngle_(angle))
    return false;
  angle[0] = wrap<float>(angle[0] - angle_offset_[0], -PI, PI);
  angle[1] = wrap<float>(angle[1] - angle_offset_[1], -PI, PI);
  angle[2] = wrap<float>(angle[2] - angle_offset_[2], -PI, PI);
  return true;
}

bool WT901::GetQuaternion(float* Q) {
  Quaternion Q_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, Q0, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&Q_, 8, 100) == HAL_OK) {
    for (int i = 0; i < 4; ++i)
      Q[i] = Q_.Q[i] / 32768.0;
    return true;
  }
  return false;
}

bool WT901::GetAcc(float* acc) {
  Acc acc_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, AX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&acc_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i)
      acc[i] = acc_.a[i] / 32768.0 * 16;
    return true;
  } 
  return false;
}

bool WT901::GetGyro(float* gyro) {
  Gyro gyro_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, GX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&gyro_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i)
      gyro[i] = gyro_.w[i] / 32768.0 * 2000;
    return true;
  }
  return false;
}

bool WT901::GetMag(int* mag) {
  Mag mag_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, HX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&mag_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i)
      mag[i] = (int)mag_.h[i];
    return true;
  } 
  return false;
}

bool WT901::SetAngleOffset() { return GetRawAngle_(angle_offset_); }

bool WT901::GetRawAngle_(float *angle) {
  Angle angle_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, Roll, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&angle_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i)
      angle[i] = (float)angle_.Angle[i] / 32768 * PI;
    return true;
  }
  return false;
}

}
