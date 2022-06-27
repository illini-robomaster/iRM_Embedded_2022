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
    case GIMBAL_FORTRESS:
      data_.pitch_offset_ = 5.205564f;
      data_.yaw_offset_ = 1.8132f;
      data_.pitch_max_ = 0.5080f;
      data_.yaw_max_ = PI;
      {
        float pitch_theta_max_iout = 0;
        float pitch_theta_max_out = 10;
        float pitch_omega_max_iout = 10000;
        float pitch_omega_max_out = 30000;
        float yaw_theta_max_iout = 0;
        float yaw_theta_max_out = 10;
        float yaw_omega_max_iout = 25000;  // 10000
        float yaw_omega_max_out = 30000;
        pitch_theta_pid_param_ = new float[3]{15, 0, 0};
        pitch_omega_pid_param_ = new float[3]{2900, 60, 0};
        yaw_theta_pid_param_ = new float[3]{26, 0, 0.3};
        yaw_omega_pid_param_ = new float[3]{3600, 20, 0};
        pitch_theta_pid_ =
            new ConstrainedPID(pitch_theta_pid_param_, pitch_theta_max_iout, pitch_theta_max_out);
        pitch_omega_pid_ =
            new ConstrainedPID(pitch_omega_pid_param_, pitch_omega_max_iout, pitch_omega_max_out);
        yaw_theta_pid_ =
            new ConstrainedPID(yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
        yaw_omega_pid_ =
            new ConstrainedPID(yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);
      }
      break;
    case GIMBAL_SENTRY:
      data_.pitch_offset_ = 1.6352f;
      data_.yaw_offset_ = 2.8718f;
      data_.pitch_max_ = 0.5080f;
      data_.yaw_max_ = PI;
      {
        float pitch_theta_max_iout = 0;
        float pitch_theta_max_out = 10;
        float pitch_omega_max_iout = 10000;
        float pitch_omega_max_out = 30000;
        float yaw_theta_max_iout = 0;
        float yaw_theta_max_out = 10;
        float yaw_omega_max_iout = 10000;
        float yaw_omega_max_out = 30000;
        pitch_theta_pid_param_ = new float[3]{15, 0, 0};
        pitch_omega_pid_param_ = new float[3]{1740, 25, 0};
        yaw_theta_pid_param_ = new float[3]{26, 0, 0.3};
        yaw_omega_pid_param_ = new float[3]{2160, 12, 0};
        pitch_theta_pid_ =
            new ConstrainedPID(pitch_theta_pid_param_, pitch_theta_max_iout, pitch_theta_max_out);
        pitch_omega_pid_ =
            new ConstrainedPID(pitch_omega_pid_param_, pitch_omega_max_iout, pitch_omega_max_out);
        yaw_theta_pid_ =
            new ConstrainedPID(yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
        yaw_omega_pid_ =
            new ConstrainedPID(yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);
      }
      break;
    case GIMBAL_STEERING:
      data_.pitch_offset_ = 5.25f;
      data_.yaw_offset_ = 0.3451;
      data_.pitch_max_ = 0.5080f;
      data_.yaw_max_ = PI;
      {
        float pitch_theta_max_iout = 0;
        float pitch_theta_max_out = 10;
        float pitch_omega_max_iout = 10000;
        float pitch_omega_max_out = 30000;
        float yaw_theta_max_iout = 0;
        float yaw_theta_max_out = 10;
        float yaw_omega_max_iout = 10000;  // 10000
        float yaw_omega_max_out = 30000;
        pitch_theta_pid_param_ = new float[3]{20, 0, 0};
        pitch_omega_pid_param_ = new float[3]{2900, 60, 0};
        yaw_theta_pid_param_ = new float[3]{30, 0, 0.3};
        yaw_omega_pid_param_ = new float[3]{3600, 20, 0};
        pitch_theta_pid_ =
            new ConstrainedPID(pitch_theta_pid_param_, pitch_theta_max_iout, pitch_theta_max_out);
        pitch_omega_pid_ =
            new ConstrainedPID(pitch_omega_pid_param_, pitch_omega_max_iout, pitch_omega_max_out);
        yaw_theta_pid_ =
            new ConstrainedPID(yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
        yaw_omega_pid_ =
            new ConstrainedPID(yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);
      }
      break;
    default:
      RM_ASSERT_TRUE(false, "Not Supported Gimbal Mode\r\n");
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
  float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);

  float yt_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
  float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
  float yt_in = yaw_motor_->GetOmegaDelta(yt_out);
  float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yt_in);

  pitch_motor_->SetOutput(po_out);
  yaw_motor_->SetOutput(yo_out);
}

void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
  float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
  float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
  pitch_angle_ = wrap<float>(clipped_pitch + data_.pitch_offset_, 0, 2 * PI);
  yaw_angle_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0, 2 * PI);
}

void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
  pitch_angle_ = pitch_motor_->GetTheta() + rel_pitch;
  yaw_angle_ = yaw_motor_->GetTheta() + rel_yaw;
}

}  // namespace control
