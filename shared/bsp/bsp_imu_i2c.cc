

#include "bsp_imu_i2c.h"

namespace bsp {

IMU::IMU(I2C_HandleTypeDef* i2c, uint16_t DevAddr) {
  i2c_ = i2c;
  DevAddr_ = DevAddr << 1;
}

bool IMU::IsRead() {
  if (HAL_I2C_IsDeviceReady(i2c_, DevAddr_, 1, 100) == HAL_OK) {
    return true;
  } else {
    return false;
  }
}

bool IMU::GetAngle(float *angle, bool is_degree) {
  Angle angle_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, Roll, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&angle_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i) {
      angle[i] = (float)angle_.Angle[i] / 32768;
      if (is_degree) {
        angle[i] *= 180;
      } else {
        angle[i] *= PI;
      }
    }
    return true;
  } else {
    return false;
  }
}

}
