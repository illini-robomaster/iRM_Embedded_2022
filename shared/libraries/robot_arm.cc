#include "robot_arm.h"

namespace control {
  Robot_arm::Robot_arm(const robot_arm_t robot_arm) : pids_() {
    model_ = robot_arm.model;

    switch(model_) {
      case ROBOT_ARM_STANDARD_ZERO:
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
        break;
      default:
        RM_ASSERT_TRUE(false, "No robot_arm type specified");
    }

  }
  Robot_arm::~Robot_arm() {
    switch (model_) {
      case ROBOT_ARM_STANDARD_ZERO:
        motors_[Motors::left] = nullptr;
        motors_[Motors::right] = nullptr;
        motors_[Motors::gripper] = nullptr;
        delete[] motors_;
        motors_ = nullptr;

        delete[] speeds_;
        speeds_ = nullptr;
        break;
    }
  }
}
