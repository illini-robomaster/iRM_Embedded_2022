#include "gimbal.h"
#include "motor.h"
#include "utils.h"

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;

namespace control {

// PID parameters lagecy gimbal
//  pid_pitch(PIDController(10, 0.25, 0.15))
//  pid_yaw(PIDController(10, 0.15, 0.15))

Gimbal::Gimbal() : pid_pitch(PIDController(10, 0.25, 0.15)),
                   pid_yaw(PIDController(10, 0.15, 0.15)),
                   pid_pluck(PIDController(20, 10, 5)) {
  can = new bsp::CAN(&hcan1, 0x205);
  // GM6020_START_ID = 0x205;
  motor_pitch = new Motor6020(can, 0x205);
  motor_yaw = new Motor6020(can, 0x206);
  motor_pluck = new Motor3508(can, 0x207);
  motors = new MotorCANBase*[3];
  motors[0] = motor_pitch;
  motors[1] = motor_yaw;
  motors[2] = motor_pluck;
  pitch_angle = pitch_offset;
  yaw_angle = yaw_offset;
  motor1 = new control::MotorPWMBase(&htim1, 1, 1000000, 500, 1080);
  motor2 = new control::MotorPWMBase(&htim1, 4, 1000000, 500, 1080);
  pluck_command = new BoolEdgeDetecter(false);
}

void Gimbal::move() {
  constexpr float multiplier = 30000 / PI;
  float pitch_diff = motor_pitch->GetThetaDelta(pitch_angle);
  float yaw_diff = motor_yaw->GetThetaDelta(yaw_angle);
  motor_pitch->SetOutput(multiplier * pid_pitch.ComputeOutput(pitch_diff));
  motor_yaw->SetOutput(multiplier * pid_yaw.ComputeOutput(yaw_diff));
  MotorCANBase::TransmitOutput(motors, 3);
}

void Gimbal::target(float new_pitch, float new_yaw) {
  pitch_angle = clip<float>(new_pitch, -PI / 2, PI / 2) + pitch_offset;
  yaw_angle = clip<float>(new_yaw, -PI / 4, PI / 4) + yaw_offset;
}

void Gimbal::shoot(bool status) {
  if (status) {
    motor1->SetOutput(100);
    motor2->SetOutput(100);
  } else {
    motor1->SetOutput(0);
    motor2->SetOutput(0);
  }
}
void Gimbal::pluck(bool status) {
  pluck_command->input(status);
  static int target_speed = 0;
  if (pluck_command->posEdge()) {
    pid_pluck.Reset();
    target_speed = 80;
  } else if (pluck_command->negEdge()) {
    pid_pluck.Reset();
    target_speed = 0;
  }
  float pluck_diff = motor_pluck->GetOmegaDelta(target_speed);
  motor_pluck->SetOutput(pid_pluck.ComputeOutput(pluck_diff));
}
    
} // namespace control
