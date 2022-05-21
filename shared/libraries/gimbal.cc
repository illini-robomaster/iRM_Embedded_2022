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
      {
        float pitch_theta_max_iout = 0;
        float pitch_theta_max_out = 0;
        float pitch_omega_max_iout = 0;
        float pitch_omega_max_out = 0;
        float yaw_theta_max_iout = 0;
        float yaw_theta_max_out = 0;
        float yaw_omega_max_iout = 0;
        float yaw_omega_max_out = 0;
        pitch_theta_pid_param_ = new float[3]{0, 0, 0};
        pitch_omega_pid_param_ = new float[3]{0, 0, 0};
        yaw_theta_pid_param_ = new float[3]{0, 0, 0};
        yaw_omega_pid_param_ = new float[3]{0, 0, 0};
        pitch_theta_pid_ = new ConstraintedPID(pitch_theta_pid_param_, pitch_theta_max_iout, pitch_theta_max_out);
        pitch_omega_pid_ = new ConstraintedPID(pitch_omega_pid_param_, pitch_omega_max_iout, pitch_omega_max_out);
        yaw_theta_pid_ = new ConstraintedPID(yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
        yaw_omega_pid_ = new ConstraintedPID(yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);
      }
      break;
    case GIMBAL_STANDARD_2022_ALPHA:
      data_.pitch_offset_ = 1.0500f;
      data_.yaw_offset_ = 5.2584f;
      data_.pitch_max_ = 0.5080f;
      data_.yaw_max_ = PI;
      {
        float pitch_theta_max_iout = 0;
        float pitch_theta_max_out = 2;
        float pitch_omega_max_iout = 1500;
        float pitch_omega_max_out = 7000;
        float yaw_theta_max_iout = 0;
        float yaw_theta_max_out = 1;
        float yaw_omega_max_iout = 500;
        float yaw_omega_max_out = 7000;
        pitch_theta_pid_param_ = new float[3]{35, 0, 12};
        pitch_omega_pid_param_ = new float[3]{1500, 0.05, 45};
        yaw_theta_pid_param_ = new float[3]{43, 0, 12};
        yaw_omega_pid_param_ = new float[3]{2900, 0.02, 45};
        // pitch_theta_pid_param_ = new float[3]{15, 0, 0};
        // pitch_omega_pid_param_ = new float[3]{3500, 0.1, 10};
        // yaw_theta_pid_param_ = new float[3]{18, 0, 0.3};
        // yaw_omega_pid_param_ = new float[3]{3500, 0.1, 0};
        // pitch_theta_pid_param_ = new float[3]{25, 0, 0.05};
        // pitch_omega_pid_param_ = new float[3]{1200, 0, 0};
        // yaw_theta_pid_param_ = new float[3]{38, 0, 0};
        // yaw_omega_pid_param_ = new float[3]{2600, 0, 30};
        pitch_theta_pid_ = new ConstraintedPID(pitch_theta_pid_param_, pitch_theta_max_iout, pitch_theta_max_out);
        pitch_omega_pid_ = new ConstraintedPID(pitch_omega_pid_param_, pitch_omega_max_iout, pitch_omega_max_out);
        yaw_theta_pid_ = new ConstraintedPID(yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
        yaw_omega_pid_ = new ConstraintedPID(yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);
      }
      break;
    default:
      RM_ASSERT_TRUE(false, "No gimbal type specified");
  }

  pitch_angle_ = data_.pitch_offset_;
  yaw_angle_ = data_.yaw_offset_;
}

Gimbal::~Gimbal() {
  delete pitch_theta_pid_param_;
  pitch_theta_pid_param_ = nullptr;
  delete pitch_omega_pid_param_;
  pitch_omega_pid_param_ = nullptr;
  delete yaw_theta_pid_param_;
  yaw_theta_pid_param_ = nullptr;
  delete yaw_omega_pid_param_;
  yaw_omega_pid_param_ = nullptr;
  delete pitch_theta_pid_;
  pitch_theta_pid_ = nullptr;
  delete pitch_omega_pid_;
  pitch_omega_pid_ = nullptr;
  delete yaw_theta_pid_;
  yaw_theta_pid_ = nullptr;
  delete yaw_omega_pid_;
  yaw_omega_pid_ = nullptr;
}

gimbal_data_t* Gimbal::GetData() { return &data_; }

void Gimbal::Update() {
  float pt_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
  float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);
  float po_in = pitch_motor_->GetOmegaDelta(pt_out);
  float po_out = pitch_omega_pid_->ComputeConstraintedOutput(po_in);

  float yt_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
  float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
  float yt_in = yaw_motor_->GetOmegaDelta(yt_out);
  float yo_out = yaw_omega_pid_->ComputeConstraintedOutput(yt_in);

  pitch_motor_->SetOutput(po_out);
  yaw_motor_->SetOutput(yo_out);
}

void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
  float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
  float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
  pitch_angle_ = wrap<float>(clipped_pitch + data_.pitch_offset_, 0, 2 * PI);
  // pitch_angle_ = data_.pitch_offset_;
  yaw_angle_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0, 2 * PI);
}

void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
  float abs_pitch = wrap<float>(rel_pitch + pitch_angle_ - data_.pitch_offset_, -PI, PI);
  float abs_yaw = wrap<float>(rel_yaw + yaw_angle_ - data_.yaw_offset_, -PI, PI);
  TargetAbs(abs_pitch, abs_yaw);
}

}  // namespace control
