//
// Created by neo on 2/8/22.
//

#include "robotic_arm.h"

namespace control {
RoboticArm::RoboticArm(robotic_arm_t robotic_arm) {
  servo_t servo_data[ARM_MOTOR_NUM];

  servo_data[left].motor = robotic_arm.motors[left];
  servo_data[left].mode = control::SERVO_NEAREST;
  servo_data[left].max_speed = 1 * PI;
  servo_data[left].max_acceleration = 1 * PI;
  servo_data[left].transmission_ratio = M3508P19_RATIO;
  servo_data[left].omega_pid_param = new float[3]{25, 5, 22};

  servo_data[right].motor = robotic_arm.motors[right];
  servo_data[right].mode = control::SERVO_NEAREST;
  servo_data[right].max_speed = 1 * PI;
  servo_data[right].max_acceleration = 1 * PI;
  servo_data[right].transmission_ratio = M3508P19_RATIO;
  servo_data[right].omega_pid_param = new float[3]{25, 5, 22};

  servo_data[gripper].motor = robotic_arm.motors[gripper];
  servo_data[gripper].mode = control::SERVO_NEAREST;
  servo_data[gripper].max_speed = 1 * PI;
  servo_data[gripper].max_acceleration = 1 * PI;
  servo_data[gripper].transmission_ratio = M3508P19_RATIO;
  servo_data[gripper].omega_pid_param = new float[3]{25, 5, 22};

  servo_[left] = new control::ServoMotor(servo_data[left]);
  servo_[right] = new control::ServoMotor(servo_data[right]);
  servo_[gripper] = new control::ServoMotor(servo_data[gripper]);
}

RoboticArm::~RoboticArm() {
  for (auto & i : servo_) {
    delete i;
    i = nullptr;
  }
}

void RoboticArm::SetPosition(const float arm_position, const float gripper_position) {
  servo_[left]->SetTarget(arm_position);
  servo_[right]->SetTarget(-arm_position);
  servo_[gripper]->SetTarget(gripper_position);
}

void RoboticArm::Update() {
  for (auto & i : servo_) {
    i->CalcOutput();
  }
}

}
