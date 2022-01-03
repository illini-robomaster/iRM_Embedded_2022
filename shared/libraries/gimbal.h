#pragma once

#include "controller.h"
#include "motor.h"
#include "can.h"
#include "utils.h"

namespace control {

  
/** @defgroup Transmission Ratios of DJI motors, reference to motor manuals.
* @{
*/
#define LEGACY_GIMBAL_POFF 4.725  /*!< Lagacy gimbal pitch offset */
#define LEGACY_GIMBAL_YOFF 3.406  /*!< Lagacy gimbal yaw offset   */
/**
  * @}
  */

typedef struct {
  MotorCANBase* pitch_motor;
  MotorCANBase* yaw_motor;
  float pitch_offset;
  float yaw_offset;
} gimbal_t;

class Gimbal {
  public:
    /**
     * @brief constructor for Gimbal instance
     */
    Gimbal(MotorCANBase* pitch, MotorCANBase* yaw, float pitch_offset, float yaw_offset);

    /**
     * @brief update the position of gimbal
     */
    void CalcOutput();

    /**
     * @brief set motors to point to a new orientation
     *
     * @param new_pitch new pitch angled
     * @param new_yaw   new yaw angled
     */
    void TargetAbs(float new_pitch, float new_yaw);

    /**
     * @brief set motors to point to a new orientation
     *
     * @param new_pitch new pitch angled
     * @param new_yaw   new yaw angled
     */
    void TargetRel(float new_pitch, float new_yaw);

  private:
    MotorCANBase* pitch_motor_;
    MotorCANBase* yaw_motor_;

    float pitch_offset_;
    float yaw_offset_;

    float pitch_angle_;
    float yaw_angle_;

    PIDController pitch_pid_;
    PIDController yaw_pid_;
};

} // namespace control