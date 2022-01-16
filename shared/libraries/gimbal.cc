#include "gimbal.h"

#include "motor.h"
#include "utils.h"

namespace control {

Gimbal::Gimbal(gimbal_t gimbal)
    : pitch_detector_(BoolEdgeDetector(false)), yaw_detector_(BoolEdgeDetector(false)) {
  // acquired from user
  pitch_motor_ = gimbal.pitch_motor;
  yaw_motor_ = gimbal.yaw_motor;
  model_ = gimbal.model;

  // data initialization using acquired model
  switch (gimbal.model) {
    case GIMBAL_STANDARD_ZERO:
      data_.pitch_offset_ = 4.725f;
      data_.yaw_offset_ = 3.406f;
      data_.pitch_max_ = 0.408f;
      data_.yaw_max_ = 1.551f;
      data_.pitch_proximity_ = data_.pitch_max_ / 3.0;
      data_.yaw_proximity_ = data_.yaw_max_ / 6.0;

      pitch_move_pid_param_ = new float[3]{1400, 0, 2200};
      pitch_hold_pid_param_ = new float[3]{3200, 100, 3100};
      yaw_move_pid_param_ = new float[3]{1000, 0, 2000};
      yaw_hold_pid_param_ = new float[3]{3000, 60, 2500};
      pitch_pid_ = new PIDController(pitch_move_pid_param_);
      yaw_pid_ = new PIDController(yaw_move_pid_param_);
      break;
    default:
      RM_ASSERT_TRUE(false, "No gimbal type specified");
  }

  pitch_angle_ = data_.pitch_offset_;
  yaw_angle_ = data_.yaw_offset_;
}

Gimbal::~Gimbal() {
  delete pitch_move_pid_param_;
  pitch_move_pid_param_ = nullptr;
  delete pitch_hold_pid_param_;
  pitch_hold_pid_param_ = nullptr;
  delete yaw_move_pid_param_;
  yaw_move_pid_param_ = nullptr;
  delete yaw_hold_pid_param_;
  yaw_hold_pid_param_ = nullptr;
  delete pitch_pid_;
  pitch_pid_ = nullptr;
  delete yaw_pid_;
  yaw_pid_ = nullptr;
}

gimbal_data_t Gimbal::GetData() const { return data_; }

void Gimbal::Update() {
  float pitch_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
  float yaw_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
  pitch_detector_.input(abs(pitch_diff) > data_.pitch_proximity_);
  yaw_detector_.input(abs(yaw_diff) > data_.yaw_proximity_);

  if (pitch_detector_.posEdge()) {
    pitch_pid_->Reinit(pitch_move_pid_param_);
  } else if (pitch_detector_.negEdge()) {
    pitch_pid_->Reinit(pitch_hold_pid_param_);
  }
  if (yaw_detector_.posEdge()) {
    yaw_pid_->Reinit(yaw_move_pid_param_);
  } else if (yaw_detector_.negEdge()) {
    yaw_pid_->Reinit(yaw_hold_pid_param_);
  }

  constexpr float multiplier = 50;
  pitch_motor_->SetOutput(pitch_pid_->ComputeConstraintedOutput(pitch_diff * multiplier));
  yaw_motor_->SetOutput(yaw_pid_->ComputeConstraintedOutput(yaw_diff * multiplier));
}

void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
  float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
  float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
  pitch_angle_ = wrap<float>(clipped_pitch + data_.pitch_offset_, 0, 2 * PI);
  yaw_angle_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0, 2 * PI);
}

void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
  float abs_pitch = wrap<float>(rel_pitch + pitch_angle_ - data_.pitch_offset_, -PI, PI);
  float abs_yaw = wrap<float>(rel_yaw + yaw_angle_ - data_.yaw_offset_, -PI, PI);
  TargetAbs(abs_pitch, abs_yaw);
}

}  // namespace control
