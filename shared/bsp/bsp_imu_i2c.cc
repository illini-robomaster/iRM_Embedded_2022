

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
      switch (i) {
        case 0:
          angle[0] = (float)angle_.Angle[0] / 32768;
          break;
        case 1:
          angle[1] = (float)angle_.Angle[1] / 32768;
          break;
        case 2:
          angle[2] = (float)angle_.Angle[2] / 32768;
          break;
        default: break;
      }
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

bool IMU::GetQuaternion(float* Q) {
  Quaternion Q_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, Q0, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&Q_, 8, 100) == HAL_OK) {
    for (int i = 0; i < 4; ++i) {
      Q[i] = (float)Q_.Q[i] / 32768;
    }
    return true;
  } else {
    return false;
  }
}

bool IMU::GetAcc(float* acc) {
  Acc acc_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, AX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&acc_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i) {
      acc[i] = (float)acc_.a[i] / 32768 * 16;
    }
    return true;
  } else {
    return false;
  }
}

bool IMU::GetGyro(float* gyro) {
  Gyro gyro_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, GX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&gyro_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i) {
      gyro[i] = (float)gyro_.w[i] / 32768 * 2000;
    }
    return true;
  } else {
    return false;
  }
}

bool IMU::GetMag(int* mag) {
  Mag mag_;
  if (HAL_I2C_Mem_Read(i2c_, DevAddr_, HX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&mag_, 6, 100) == HAL_OK) {
    for (int i = 0; i < 3; ++i) {
      mag[i] = (int)mag_.h[i];
    }
    return true;
  } else {
    return false;
  }
}

}
