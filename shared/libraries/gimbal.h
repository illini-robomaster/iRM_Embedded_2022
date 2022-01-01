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
     * @param new_pitch new pitch angled
     * @param new_yaw   new yaw angled
     */
    void target(float new_pitch, float new_yaw);

    void shoot_start();
    void shoot_stop();

    void pluck_start();
    void pluck_stop();

  private:
    bsp::CAN* can = NULL;
    // const float pitch_offset = 4.7251722;
    const float pitch_offset = 4.5251722;
    const float yaw_offset = 3.406;
    float pitch_angle;
    float yaw_angle;

    PIDController pid_pitch;
    PIDController pid_yaw;
    PIDController pid_pluck;

    MotorCANBase* motor_pitch;
    MotorCANBase* motor_yaw;
    MotorCANBase* motor_pluck;
    MotorCANBase** motors;

    MotorPWMBase* motor1;
    MotorPWMBase* motor2;
};

} // namespace control