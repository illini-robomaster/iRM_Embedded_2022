#include "controller.h"
#include "utils.h"

namespace control {

PIDController::PIDController(float kp, float ki, float kd) {
  pid_f32_.Kp = kp;
  pid_f32_.Ki = ki;
  pid_f32_.Kd = kd;
  arm_pid_init_f32(&pid_f32_, 1);
}

float PIDController::ComputeOutput(float error) { 
  return arm_pid_f32(&pid_f32_, error); 
}

int16_t PIDController::ComputeConstraintedOutput(float error) { 
  constexpr int MIN = -32768;
  constexpr int MAX = 32767;
  return clip<int>((int) arm_pid_f32(&pid_f32_, error), MIN, MAX); 
}

void PIDController::Reset() { 
  arm_pid_init_f32(&pid_f32_, 1);
}

} /* namespace control */
