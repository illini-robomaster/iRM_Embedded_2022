#include "gimbal.h"
#include "motor.h"
#include "utils.h"

extern CAN_HandleTypeDef hcan1;

namespace control {

Gimbal::Gimbal() : pid_pitch(PIDController(20, 8, 0.1)),
                   pid_yaw(PIDController(20, 8, 0.1)) {
  can = new bsp::CAN(&hcan1, 0x205);
  constexpr int GM6020_START_ID = 0x205;
  motor_pitch = new Motor6020(can, GM6020_START_ID);
  motor_yaw = new Motor6020(can, GM6020_START_ID + 1);
  motors = new MotorCANBase*[2];
  motors[0] = motor_pitch;
  motors[1] = motor_yaw;
  pitch_angle = pitch_offset;
  yaw_angle = yaw_offset;
}

void Gimbal::move() {
  constexpr float multiplier = 5;
  float pitch_diff = motor_pitch->GetThetaDelta(pitch_angle);
  float yaw_diff = motor_yaw->GetThetaDelta(yaw_angle);
  motor_pitch->SetOutput(multiplier * pid_pitch.ComputeOutput(pitch_diff));
  motor_yaw->SetOutput(multiplier * pid_yaw.ComputeOutput(yaw_diff));
  MotorCANBase::TransmitOutput(motors, 2);
}

void Gimbal::target(float new_pitch, float new_yaw) {
  pitch_angle = clip<float>(new_pitch, -PI / 2, PI / 2) + pitch_offset;
  yaw_angle = clip<float>(new_yaw, -PI / 4, PI / 4) + yaw_offset;
}
    
} // namespace control
