#pragma once

#include "controller.h"
#include "motor.h"
#include "can.h"

namespace control {

class Gimbal {
  public:
    /**
     * @brief constructor for Gimbal instance
     */
    Gimbal();

    /**
     * @brief update the position of gimbal
     */
    void move();

    /**
     * @brief set motors to point to a new orientation
     *
     * @param new_pitch new pitch angle
     * @param new_yaw   new yaw angle
     */
    void target(float new_pitch, float new_yaw);

  private:
    bsp::CAN* can = NULL;
    const float pitch_offset = 4.688;
    const float yaw_offset = 3.406;
    float pitch_angle;
    float yaw_angle;
    PIDController pid_pitch;
    PIDController pid_yaw;
    MotorCANBase* motor_pitch;
    MotorCANBase* motor_yaw;
    MotorCANBase** motors;
};

} // namespace control