#include "gimbal.h"
#include "motor.h"
#include "utils.h"

namespace control {

// PID parameters lagecy gimbal
//  pid_pitch(PIDController(10, 0.25, 0.15))
//  pid_yaw(PIDController(10, 0.15, 0.15))

Gimbal::Gimbal(MotorCANBase* pitch, MotorCANBase* yaw, float pitch_offset, float yaw_offset) : 
      pitch_pid_(PIDController(10, 0.25, 0.15)),
      yaw_pid_(PIDController(10, 0.15, 0.15)) {
  pitch_motor_ = pitch;
  yaw_motor_ = yaw;
  pitch_offset_ = pitch_offset;
  yaw_offset_ = yaw_offset;
  pitch_angle_ = pitch_offset;
  yaw_angle_ = yaw_offset;
}

void Gimbal::CalcOutput() {
  constexpr float multiplier = 30000 / PI;
  float pitch_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
  float yaw_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
  pitch_motor_->SetOutput(multiplier * pitch_pid_.ComputeOutput(pitch_diff));
  yaw_motor_->SetOutput(multiplier * yaw_pid_.ComputeOutput(yaw_diff));
}

void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
  pitch_angle_ = wrap<float>(clip<float>(abs_pitch, -PI / 2, PI / 2) + pitch_offset_, -PI, PI);
  yaw_angle_ = wrap<float>(clip<float>(abs_yaw, -PI / 4, PI / 4) + yaw_offset_, -PI, PI);
}

void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
  pitch_angle_ = wrap<float>(clip<float>(rel_pitch, -PI / 2, PI / 2) + pitch_offset_, -PI, PI);
  yaw_angle_ = wrap<float>(clip<float>(rel_yaw, -PI / 4, PI / 4) + yaw_offset_, -PI, PI);
}
    
} // namespace control
