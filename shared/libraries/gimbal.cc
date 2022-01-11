#include "gimbal.h"
#include "motor.h"
#include "utils.h"

namespace control {

Gimbal::Gimbal(gimbal_t gimbal) : 
    pitch_detector_(BoolEdgeDetector(false)), yaw_detector_(BoolEdgeDetector(false)) {
  pitch_motor_ = gimbal.pitch_motor;
  yaw_motor_ = gimbal.yaw_motor;

#if defined(GIMBAL_2019)
  pitch_offset_ = GIMBAL_2019_POFF;
  yaw_offset_ = GIMBAL_2019_YOFF;
  pitch_max_ = GIMBAL_2019_PMAX;
  yaw_max_ = GIMBAL_2019_YMAX;
  pitch_proximity_ = GIMBAL_2019_PMAX / 3.0;
  yaw_proximity_ = GIMBAL_2019_YMAX / 6.0;

  pitch_move_pid_param_ = new float[3] {1000, 0, 100};
  pitch_hold_pid_param_ = new float[3] {2000, 100, 100};
  yaw_move_pid_param_ = new float[3] {600, 0, 100};
  yaw_hold_pid_param_ = new float[3] {1500, 15, 200};
  pitch_pid_ = new PIDController(pid_move_param_);
  yaw_pid_ = new PIDController(pid_hold_param_);
#else
  RM_ASSERT_TRUE(false, "No gimbal type specified");
#endif

  pitch_angle_ = pitch_offset_;
  yaw_angle_ = yaw_offset_;
}

Gimbal::~Gimbal() {
  delete pitch_move_pid_param_;
  delete pitch_hold_pid_param_;
  delete yaw_move_pid_param_;
  delete yaw_hold_pid_param_;
  delete pitch_pid_;
  delete yaw_pid_;
}

void Gimbal::Update() {
  float pitch_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
  float yaw_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
  pitch_detector_.input(abs(pitch_diff) > pitch_proximity_);
  yaw_detector_.input(abs(yaw_diff) > yaw_proximity_);

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
  pitch_angle_ = wrap<float>(clip<float>(abs_pitch, -pitch_max_, pitch_max_) + pitch_offset_, 0, 2 * PI);
  yaw_angle_ = wrap<float>(clip<float>(abs_yaw, -yaw_max_, yaw_max_) + yaw_offset_, 0, 2 * PI);
}

void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
  float abs_pitch = wrap<float>(rel_pitch + pitch_angle_- pitch_offset_, 0, 2 * PI);
  float abs_yaw = wrap<float>(rel_yaw + yaw_angle_- yaw_offset_, 0, 2 * PI);
  TargetAbs(abs_pitch, abs_yaw);
  RM_EXPECT_TRUE(false, "Invalid complementary filter weight");
}
    
} // namespace control
