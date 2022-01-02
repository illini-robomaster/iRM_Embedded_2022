#pragma once

#include "controller.h"
#include "motor.h"
#include "can.h"
#include "utils.h"

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
     * @param new_pitch new pitch angled
     * @param new_yaw   new yaw angled
     */
    void target(float new_pitch, float new_yaw);

    void shoot(bool status);
    void pluck(bool status);

  private:
#ifdef LEGACY_GIMBAL
    const float pitch_offset = 4.7251722;
    const float yaw_offset = 3.406;
#else
    const float pitch_offset = 0;
    const float yaw_offset = 0;
#endif
    float pitch_angle;
    float yaw_angle;

    PIDController pid_pitch;
    PIDController pid_yaw;
    PIDController pid_pluck;

    bsp::CAN* can;
    MotorCANBase* motor_pitch;
    MotorCANBase* motor_yaw;
    MotorCANBase* motor_pluck;
    MotorCANBase** motors;

    MotorPWMBase* motor1;
    MotorPWMBase* motor2;

    BoolEdgeDetecter* pluck_command;
};

} // namespace control