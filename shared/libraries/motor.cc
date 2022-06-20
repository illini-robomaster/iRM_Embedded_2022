/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "motor.h"

#include "arm_math.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "utils.h"

using namespace bsp;

namespace control {

int16_t ClipMotorRange(float output) {
  constexpr int MIN = -MOTOR_RANGE; /* Minimum that a 16-bit number can represent */
  constexpr int MAX = MOTOR_RANGE;  /* Maximum that a 16-bit number can represent */
  return (int16_t)clip<int>((int)output, MIN, MAX);
}

/**
 * @brief standard can motor callback, used to update motor data
 *
 * @param data data that come from motor
 * @param args pointer to a MotorCANBase instance
 */
static void can_motor_callback(const uint8_t data[], void* args) {
  MotorCANBase* motor = reinterpret_cast<MotorCANBase*>(args);
  motor->UpdateData(data);
}

MotorCANBase::MotorCANBase(bsp::CAN* can, uint16_t rx_id)
    : theta_(0), omega_(0), can_(can), rx_id_(rx_id) {
  constexpr uint16_t GROUP_SIZE = 4;
  constexpr uint16_t RX1_ID_START = 0x201;
  constexpr uint16_t RX2_ID_START = 0x205;
  constexpr uint16_t RX3_ID_START = 0x209;
  constexpr uint16_t TX1_ID = 0x200;
  constexpr uint16_t TX2_ID = 0x1ff;
  constexpr uint16_t TX3_ID = 0x2ff;

  RM_ASSERT_GE(rx_id, RX1_ID_START, "Invalid rx id");
  RM_ASSERT_LT(rx_id, RX3_ID_START + GROUP_SIZE, "Invalid rx id");
  if (rx_id >= RX3_ID_START)
    tx_id_ = TX3_ID;
  else if (rx_id >= RX2_ID_START)
    tx_id_ = TX2_ID;
  else
    tx_id_ = TX1_ID;
}

void MotorCANBase::TransmitOutput(MotorCANBase* motors[], uint8_t num_motors) {
  uint8_t data[8] = {0};

  RM_ASSERT_GT(num_motors, 0, "Meaningless empty can motor transmission");
  RM_ASSERT_LE(num_motors, 4, "Exceeding maximum of 4 motor commands per CAN message");
  for (uint8_t i = 0; i < num_motors; ++i) {
    RM_ASSERT_EQ(motors[i]->tx_id_, motors[0]->tx_id_, "tx id mismatch");
    RM_ASSERT_EQ(motors[i]->can_, motors[0]->can_, "can line mismatch");
    const uint8_t motor_idx = (motors[i]->rx_id_ - 1) % 4;
    const int16_t output = motors[i]->output_;
    data[2 * motor_idx] = output >> 8;
    data[2 * motor_idx + 1] = output & 0xff;
  }

  motors[0]->can_->Transmit(motors[0]->tx_id_, data, 8);
}

float MotorCANBase::GetTheta() const { return theta_; }

float MotorCANBase::GetThetaDelta(float target) const {
  return wrap<float>(target - theta_, -PI, PI);
}

float MotorCANBase::GetOmega() const { return omega_; }

float MotorCANBase::GetOmegaDelta(float target) const { return target - omega_; }

int16_t MotorCANBase::GetCurr() const { return 0; }

uint16_t MotorCANBase::GetTemp() const { return 0; }

Motor3508::Motor3508(CAN* can, uint16_t rx_id) : MotorCANBase(can, rx_id) {
  can->RegisterRxCallback(rx_id, can_motor_callback, this);
}

void Motor3508::UpdateData(const uint8_t data[]) {
  const int16_t raw_theta = data[0] << 8 | data[1];
  const int16_t raw_omega = data[2] << 8 | data[3];
  raw_current_get_ = data[4] << 8 | data[5];
  raw_temperature_ = data[6];

  constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
  constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
  theta_ = raw_theta * THETA_SCALE;
  omega_ = raw_omega * OMEGA_SCALE;

  connection_flag_ = true;
}

void Motor3508::PrintData() const {
  print("theta: % .4f ", GetTheta());
  print("omega: % .4f ", GetOmega());
  print("raw temperature: %3d ", raw_temperature_);
  print("raw current get: % d \r\n", raw_current_get_);
}

void Motor3508::SetOutput(int16_t val) {
  constexpr int16_t MAX_ABS_CURRENT = 12288;  // ~20A
  output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
}

int16_t Motor3508::GetCurr() const { return raw_current_get_; }

uint16_t Motor3508::GetTemp() const { return raw_temperature_; }

Motor6020::Motor6020(CAN* can, uint16_t rx_id) : MotorCANBase(can, rx_id) {
  can->RegisterRxCallback(rx_id, can_motor_callback, this);
}

void Motor6020::UpdateData(const uint8_t data[]) {
  const int16_t raw_theta = data[0] << 8 | data[1];
  const int16_t raw_omega = data[2] << 8 | data[3];
  raw_current_get_ = data[4] << 8 | data[5];
  raw_temperature_ = data[6];

  constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
  constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
  theta_ = raw_theta * THETA_SCALE;
  omega_ = raw_omega * OMEGA_SCALE;

  connection_flag_ = true;
}

void Motor6020::PrintData() const {
  print("theta: % .4f ", GetTheta());
  print("omega: % .4f ", GetOmega());
  print("raw temperature: %3d ", raw_temperature_);
  print("raw current get: % d \r\n", raw_current_get_);
}

void Motor6020::SetOutput(int16_t val) {
  constexpr int16_t MAX_ABS_CURRENT = 30000;  // ~
  output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
}

int16_t Motor6020::GetCurr() const { return raw_current_get_; }

uint16_t Motor6020::GetTemp() const { return raw_temperature_; }

Motor6623::Motor6623(CAN* can, uint16_t rx_id) : MotorCANBase(can, rx_id) {
  can->RegisterRxCallback(rx_id, can_motor_callback, this);
}

void Motor6623::UpdateData(const uint8_t data[]) {
  const int16_t raw_theta = data[0] << 8 | data[1];
  raw_current_get_ = (data[2] << 8 | data[3]) * CURRENT_CORRECTION;
  raw_current_set_ = (data[4] << 8 | data[5]) * CURRENT_CORRECTION;

  constexpr float THETA_SCALE = 2 * PI / 8192;
  theta_ = raw_theta * THETA_SCALE;

  connection_flag_ = true;
}

void Motor6623::PrintData() const {
  print("theta: % .4f ", GetTheta());
  print("raw current get: % d ", raw_current_get_);
  print("raw current set: % d \r\n", raw_current_set_);
}

void Motor6623::SetOutput(int16_t val) {
  constexpr int16_t MAX_ABS_CURRENT = 5000;  // ~5.3A
  output_ = clip<int16_t>(val * CURRENT_CORRECTION, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
}

float Motor6623::GetOmega() const {
  RM_EXPECT_TRUE(false, "6623 does not support omega messurement");
  return 0;
}

float Motor6623::GetOmegaDelta(const float target) const {
  UNUSED(target);
  RM_EXPECT_TRUE(false, "6623 does not support omega messurement");
  return 0;
}

Motor2006::Motor2006(CAN* can, uint16_t rx_id) : MotorCANBase(can, rx_id) {
  can->RegisterRxCallback(rx_id, can_motor_callback, this);
}

void Motor2006::UpdateData(const uint8_t data[]) {
  const int16_t raw_theta = data[0] << 8 | data[1];
  const int16_t raw_omega = data[2] << 8 | data[3];
  raw_current_get_ = data[4] << 8 | data[5];

  constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
  constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
  theta_ = raw_theta * THETA_SCALE;
  omega_ = raw_omega * OMEGA_SCALE;

  connection_flag_ = true;
}

void Motor2006::PrintData() const {
  print("theta: % .4f ", GetTheta());
  print("omega: % .4f ", GetOmega());
  print("raw current get: % d \r\n", raw_current_get_);
}

void Motor2006::SetOutput(int16_t val) {
  constexpr int16_t MAX_ABS_CURRENT = 10000;  // ~10A
  output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
}

int16_t Motor2006::GetCurr() const { return raw_current_get_; }

MotorPWMBase::MotorPWMBase(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                           uint32_t output_freq, uint32_t idle_throttle)
    : pwm_(htim, channel, clock_freq, output_freq, idle_throttle), idle_throttle_(idle_throttle) {
  pwm_.Start();
}

void MotorPWMBase::SetOutput(int16_t val) {
  output_ = val;
  pwm_.SetPulseWidth(val + idle_throttle_);
}

void Motor2305::SetOutput(int16_t val) {
  constexpr int16_t MIN_OUTPUT = 0;
  constexpr int16_t MAX_OUTPUT = 700;
  MotorPWMBase::SetOutput(clip<int16_t>(val, MIN_OUTPUT, MAX_OUTPUT));
}

/**
 * @brief default servomotor callback that overrides the standard can motor callback
 *
 * @param data data that come from motor
 * @param args pointer to a ServoMotor instance
 */
static void servomotor_callback(const uint8_t data[], void* args) {
  ServoMotor* servo = reinterpret_cast<ServoMotor*>(args);
  servo->UpdateData(data);
}

ServoMotor::ServoMotor(servo_t data, float align_angle, float proximity_in, float proximity_out) {
  motor_ = data.motor;
  max_speed_ = data.transmission_ratio * data.max_speed;
  max_acceleration_ = data.transmission_ratio * data.max_acceleration;
  transmission_ratio_ = data.transmission_ratio;
  proximity_in_ = proximity_in;
  proximity_out_ = proximity_out;

  hold_ = true;
  target_angle_ = 0;
  align_angle_ = align_angle;  // Wait for Update to initialize
  motor_angle_ = 0;
  offset_angle_ = 0;
  servo_angle_ = 0;
  cumulated_angle_ = 0;
  inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
  outer_wrap_detector_ = new FloatEdgeDetector(0, PI);
  hold_detector_ = new BoolEdgeDetector(false);

  omega_pid_.Reinit(data.omega_pid_param, data.max_iout, data.max_out);

  // override origianal motor rx callback with servomotor callback
  data.motor->can_->RegisterRxCallback(data.motor->rx_id_, servomotor_callback, this);

  // Initially jam detection is not enabled, it is enabled only if user calls
  // RegisterJamCallback in the future.
  jam_callback_ = nullptr;
  detect_head_ = -1;
  detect_period_ = -1;
  detect_total_ = 0;
  detect_buf_ = nullptr;
}

servo_status_t ServoMotor::SetTarget(const float target, bool override) {
  if (!hold_ && !override) return INPUT_REJECT;
  servo_status_t dir = target < target_angle_ ? TURNING_ANTICLOCKWISE : TURNING_CLOCKWISE;
  target_angle_ = target;
  return dir;
}

void ServoMotor::SetMaxSpeed(const float max_speed) {
  if (max_speed > 0)
    max_speed_ = transmission_ratio_ * max_speed;
  else
    RM_EXPECT_TRUE(false, "Max speed should be positive");
}

void ServoMotor::SetMaxAcceleration(const float max_acceleration) {
  if (max_acceleration > 0)
    max_acceleration_ = transmission_ratio_ * max_acceleration;
  else
    RM_EXPECT_TRUE(false, "Max acceleration should be positive");
}

void ServoMotor::CalcOutput() {
  // if holding status toggle, reseting corresponding pid to avoid error building up
  hold_detector_->input(hold_);
  if (hold_detector_->edge()) omega_pid_.Reset();
  if (hold_detector_->negEdge()) start_time_ = GetHighresTickMicroSec();

  // calculate desired output with pid
  int16_t command;
  float target_diff = (target_angle_ - servo_angle_ - cumulated_angle_) * transmission_ratio_;
  // v = sqrt(2 * a * d)
  uint32_t current_time = GetHighresTickMicroSec();
  if (!hold_) {
    float speed_max_start =
        (current_time - start_time_) / 10e6 * max_acceleration_ * transmission_ratio_;
    float speed_max_target = sqrt(2 * max_acceleration_ * abs(target_diff));
    float current_speed = speed_max_start > speed_max_target ? speed_max_target : speed_max_start;
    current_speed = clip<float>(current_speed, 0, max_speed_);
    command = omega_pid_.ComputeConstrainedOutput(
        motor_->GetOmegaDelta(sign<float>(target_diff, 0) * current_speed));
  } else {
    command = omega_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(target_diff * 50));
  }
  motor_->SetOutput(command);

  // jam detection mechanism
  if (detect_buf_ != nullptr) {
    // update rolling sum and circular buffer
    detect_total_ += command - detect_buf_[detect_head_];
    detect_buf_[detect_head_] = command;
    detect_head_ = detect_head_ + 1 < detect_period_ ? detect_head_ + 1 : 0;

    // detect if motor is jammed
    jam_detector_->input(abs(detect_total_) >= jam_threshold_);
    if (jam_detector_->posEdge()) {
      servo_jam_t data;
      data.speed = max_speed_ / transmission_ratio_;
      jam_callback_(this, data);
    }
  }
}

bool ServoMotor::Holding() const { return hold_; }

float ServoMotor::GetTarget() const { return target_angle_; }

void ServoMotor::RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                                     uint8_t detect_period) {
  constexpr int maximum_command = 32768;  // maximum command that a CAN motor can accept
  RM_ASSERT_TRUE(effort_threshold > 0 && effort_threshold <= 1,
                 "Effort threshold should between 0 and 1");
  // storing function pointer for future invocation
  jam_callback_ = callback;

  // create and initialize circular buffer
  detect_head_ = 0;
  detect_period_ = detect_period;
  detect_total_ = 0;
  if (detect_buf_ != nullptr) delete detect_buf_;
  detect_buf_ = new int16_t[detect_period];
  memset(detect_buf_, 0, detect_period);

  // calculate callback trigger threshold and triggering facility
  jam_threshold_ = maximum_command * effort_threshold * detect_period;
  jam_detector_ = new BoolEdgeDetector(false);
}

void ServoMotor::PrintData() const {
  print("theta: % 9.4f ", GetTheta());
  print("omega: % 9.4f ", GetOmega());
  print("target: % 9.4f ", target_angle_);
  if (hold_)
    print("status: holding ");
  else
    print("status: moving  ");
  motor_->PrintData();
}

float ServoMotor::GetTheta() const { return servo_angle_ + cumulated_angle_; }

float ServoMotor::GetThetaDelta(const float target) const { return target - GetTheta(); }

float ServoMotor::GetOmega() const { return motor_->omega_ / transmission_ratio_; }

float ServoMotor::GetOmegaDelta(const float target) const {
  return target - motor_->GetOmega() / transmission_ratio_;
}

void ServoMotor::UpdateData(const uint8_t data[]) {
  motor_->UpdateData(data);

  // TODO: change the align angle calibration method
  // This is a dumb method to get the align angle
  if (align_angle_ < 0) align_angle_ = motor_->theta_;

  motor_angle_ = motor_->theta_ - align_angle_;
  // If motor angle is jumped from near 2PI to near 0, then wrap detecter will sense a negative
  // edge, which means that the motor is turning in positive direction when crossing encoder
  // boarder. Vice versa for motor angle jumped from near 0 to near 2PI
  inner_wrap_detector_->input(motor_angle_);
  if (inner_wrap_detector_->negEdge())
    offset_angle_ = wrap<float>(offset_angle_ + 2 * PI / transmission_ratio_, 0, 2 * PI);
  else if (inner_wrap_detector_->posEdge())
    offset_angle_ = wrap<float>(offset_angle_ - 2 * PI / transmission_ratio_, 0, 2 * PI);
  servo_angle_ = wrap<float>(offset_angle_ + motor_angle_ / transmission_ratio_, 0, 2 * PI);
  outer_wrap_detector_->input(servo_angle_);
  if (outer_wrap_detector_->negEdge())
    cumulated_angle_ += 2 * PI;
  else if (outer_wrap_detector_->posEdge())
    cumulated_angle_ -= 2 * PI;

  // determine if the motor should be in hold state
  float diff = abs(GetThetaDelta(target_angle_));
  if (!hold_ && diff < proximity_in_) hold_ = true;
  if (hold_ && diff > proximity_out_) hold_ = false;
}

SteeringMotor::SteeringMotor(steering_t data) {
  servo_t servo_data;
  servo_data.motor = data.motor;
  servo_data.max_speed = data.max_speed;
  servo_data.max_acceleration = data.max_acceleration;
  servo_data.transmission_ratio = data.transmission_ratio;
  servo_data.omega_pid_param = data.omega_pid_param;
  servo_data.max_iout = data.max_iout;
  servo_data.max_out = data.max_out;
  servo_ = new ServoMotor(servo_data, data.offset_angle);

  test_speed_ = data.test_speed;
  align_detect_func = data.align_detect_func;
  align_angle_ = 0;
  align_detector = new BoolEdgeDetector(false);
}

float SteeringMotor::GetRawTheta() const { return servo_->GetTheta(); }

void SteeringMotor::PrintData() const { servo_->PrintData(); }

void SteeringMotor::TurnRelative(float angle) {
  servo_->SetTarget(servo_->GetTarget() + angle, true);
}

void SteeringMotor::TurnAbsolute(float angle) { servo_->SetTarget(angle); }

bool SteeringMotor::AlignUpdate() {
  static bool align_complete = false;

  if (align_complete) {
    servo_->SetTarget(align_angle_, true);
    servo_->CalcOutput();
    return true;
  } else if (align_detect_func()) {
    servo_->servo_angle_ = 0;
    servo_->cumulated_angle_ = 0;
    align_complete = true;
    return true;
  } else {
    servo_->motor_->SetOutput(servo_->omega_pid_.ComputeConstrainedOutput(
        servo_->motor_->GetOmegaDelta(test_speed_ * servo_->transmission_ratio_)));
  }
  return false;
}

void SteeringMotor::Update() { servo_->CalcOutput(); }

} /* namespace control */
