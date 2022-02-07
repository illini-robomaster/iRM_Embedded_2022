#include "robot_arm.h"

#include "can.h"
#include "cmath"
#include "bsp_error_handler.h"

namespace control {
RobotArm::RobotArm(const robot_arm_t robot_arm) : pids_() {
  motors_ = new MotorCANBase* [3];
  motors_[Motors::left] = robot_arm.motors[Motors::left];
  motors_[Motors::right] = robot_arm.motors[Motors::right];
  motors_[Motors::gripper] = robot_arm.motors[Motors::gripper];
  {
    float* pid_param = new float[3] {25, 5, 35};
    pids_[Motors::left].Reinit(pid_param);
    pids_[Motors::right].Reinit(pid_param);
    pids_[Motors::gripper].Reinit(pid_param);
  }

  speeds_ = new float[3];
  for (int i = 0; i < 3; i++) {
    speeds_[i] = 0;
  }

}
  RobotArm::~RobotArm() {
        motors_[Motors::left] = nullptr;
        motors_[Motors::right] = nullptr;
        motors_[Motors::gripper] = nullptr;
        delete[] motors_;
        motors_ = nullptr;

        delete[] speeds_;
        speeds_ = nullptr;
  }
  void RobotArm::SetSpeed(const float LR_speed, const float G_speed) {
    print("Setspeed Start\r\n");
    constexpr int MAX_ABS_CURRENT =12288;
    float move_sum_LR = fabs(LR_speed);
    float move_sum_G = fabs(G_speed);
    float scale_LR = move_sum_LR >= MAX_ABS_CURRENT ? MAX_ABS_CURRENT / move_sum_LR : 1;
    float scale_G = move_sum_G >= MAX_ABS_CURRENT ? MAX_ABS_CURRENT / move_sum_G : 1;

    speeds_[Motors::left] = scale_LR * (LR_speed);
    speeds_[Motors::right] = -scale_LR * (LR_speed);
    speeds_[Motors::gripper] = scale_G * (G_speed);
  }

  void RobotArm::Update() {
    motors_[Motors::left]->SetOutput(
        pids_[Motors::left].ComputeConstraintedOutput(
              motors_[Motors::left]->GetOmegaDelta(speeds_[Motors::left])));
    motors_[Motors::right]->SetOutput(
        pids_[Motors::right].ComputeConstraintedOutput(
            motors_[Motors::right]->GetOmegaDelta(speeds_[Motors::right])));
    motors_[Motors::gripper]->SetOutput(
        pids_[Motors::gripper].ComputeConstraintedOutput(
            motors_[Motors::gripper]->GetOmegaDelta(speeds_[Motors::gripper])));
  }
}
