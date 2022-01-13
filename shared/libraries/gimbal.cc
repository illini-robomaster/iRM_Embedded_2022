#include "gimbal.h"
#include "motor.h"
#include "utils.h"

namespace control {

Gimbal::Gimbal(gimbal_t gimbal) : 
      pitch_pid_(PIDController(gimbal.pitch_hold_Kp, gimbal.pitch_hold_Ki, gimbal.pitch_hold_Kd)),
      yaw_pid_(PIDController(gimbal.yaw_hold_Kp, gimbal.yaw_hold_Ki, gimbal.yaw_hold_Kd)),
      pitch_detector_(BoolEdgeDetector(false)),
      yaw_detector_(BoolEdgeDetector(false)) {
  pitch_motor_ = gimbal.pitch_motor;
  yaw_motor_ = gimbal.yaw_motor;
  pitch_offset_ = gimbal.pitch_offset;
  yaw_offset_ = gimbal.yaw_offset;
  pitch_max_ = gimbal.pitch_max;
  yaw_max_ = gimbal.yaw_max;
  pitch_proximity_ = gimbal.pitch_proximity;
  yaw_proximity_ = gimbal.yaw_proximity;

  pitch_move_Kp_ = gimbal.pitch_move_Kp;
  pitch_move_Ki_ = gimbal.pitch_move_Ki;
  pitch_move_Kd_ = gimbal.pitch_move_Kd;
  yaw_move_Kp_ = gimbal.yaw_move_Kp;
  yaw_move_Ki_ = gimbal.yaw_move_Ki;
  yaw_move_Kd_ = gimbal.yaw_move_Kd;
  pitch_hold_Kp_ = gimbal.pitch_hold_Kp;
  pitch_hold_Ki_ = gimbal.pitch_hold_Ki;
  pitch_hold_Kd_ = gimbal.pitch_hold_Kd;
  yaw_hold_Kp_ = gimbal.yaw_hold_Kp;
  yaw_hold_Ki_ = gimbal.yaw_hold_Ki;
  yaw_hold_Kd_ = gimbal.yaw_hold_Kd;

  pitch_angle_ = gimbal.pitch_offset;
  yaw_angle_ = gimbal.yaw_offset;
}

void Gimbal::CalcOutput() {
  float pitch_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
  float yaw_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
  pitch_detector_.input(abs(pitch_diff) > pitch_proximity_);
  yaw_detector_.input(abs(yaw_diff) > yaw_proximity_);

  if (pitch_detector_.posEdge()) {
    pitch_pid_.Reinit(pitch_move_Kp_, pitch_move_Ki_, pitch_move_Kd_);
  } else if (pitch_detector_.negEdge()) {
    pitch_pid_.Reinit(pitch_hold_Kp_, pitch_hold_Ki_, pitch_hold_Kd_);
  }
  if (yaw_detector_.posEdge()) {
    yaw_pid_.Reinit(yaw_move_Kp_, yaw_move_Ki_, yaw_move_Kd_);
  } else if (yaw_detector_.negEdge()) {
    yaw_pid_.Reinit(yaw_hold_Kp_, yaw_hold_Ki_, yaw_hold_Kd_);
  }

  constexpr float multiplier = 50;
  pitch_motor_->SetOutput(pitch_pid_.ComputeConstraintedOutput(pitch_diff * multiplier));
  yaw_motor_->SetOutput(yaw_pid_.ComputeConstraintedOutput(yaw_diff * multiplier));
}

void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
  pitch_angle_ = wrap<float>(clip<float>(abs_pitch, -pitch_max_, pitch_max_) + pitch_offset_, -PI, PI);
  yaw_angle_ = wrap<float>(clip<float>(abs_yaw, -yaw_max_, yaw_max_) + yaw_offset_, -PI, PI);
}

void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
  float abs_pitch = wrap<float>(rel_pitch + pitch_angle_- pitch_offset_, -PI, PI);
  float abs_yaw = wrap<float>(rel_yaw + yaw_angle_- yaw_offset_, -PI, PI);
  TargetAbs(abs_pitch, abs_yaw);
}
    
} // namespace control
