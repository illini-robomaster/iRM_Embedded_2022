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

#pragma once

#include "bsp_can.h"
#include "bsp_pwm.h"
#include "controller.h"
#include "utils.h"

namespace control {

/**
 * @brief base class for motor representation
 */
class MotorBase {
 public:
  MotorBase() : output_(0) {}
  virtual ~MotorBase() {}

  virtual void SetOutput(int16_t val) { output_ = val; }

 protected:
  int16_t output_;
};

/**
 * @brief base class for CAN motor representation
 */
class MotorCANBase : public MotorBase {
 public:
  /**
   * @brief base constructor
   *
   * @param can    CAN instance
   * @param rx_id  CAN rx id
   */
  MotorCANBase(bsp::CAN* can, uint16_t rx_id);

  /**
   * @brief update motor feedback data
   * @note only used in CAN callback, do not call elsewhere
   *
   * @param data[]  raw data bytes
   */
  virtual void UpdateData(const uint8_t data[]) = 0;

  /**
   * @brief print out motor data
   */
  virtual void PrintData() const = 0;

  /**
   * @brief get motor angle, in [rad]
   *
   * @return radian angle, range between [0, 2PI]
   */
  virtual float GetTheta() const;

  /**
   * @brief get angle difference (target - actual), in [rad]
   *
   * @param target  target angle, in [rad]
   *
   * @return angle difference, range between [-PI, PI]
   */
  virtual float GetThetaDelta(const float target) const;

  /**
   * @brief get angular velocity, in [rad / s]
   *
   * @return angular velocity
   */
  virtual float GetOmega() const;

  /**
   * @brief get angular velocity difference (target - actual), in [rad / s]
   *
   * @param target  target angular velocity, in [rad / s]
   *
   * @return difference angular velocity
   */
  virtual float GetOmegaDelta(const float target) const;

  /**
   * @brief transmit CAN message for setting motor outputs
   *
   * @param motors[]    array of CAN motor pointers
   * @param num_motors  number of motors to transmit
   */
  static void TransmitOutput(MotorCANBase* motors[], uint8_t num_motors);

  /**
   * @brief set ServoMotor as friend of MotorCANBase since they need to use
   *        many of the private parameters of MotorCANBase.
   */
  friend class ServoMotor;

 protected:
  volatile float theta_;
  volatile float omega_;

 private:
  bsp::CAN* can_;
  uint16_t rx_id_;
  uint16_t tx_id_;
};

/**
 * @brief DJI 2006 motor class
 */
class Motor2006 : public MotorCANBase {
 public:
  /* constructor wrapper over MotorCANBase */
  Motor2006(bsp::CAN* can, uint16_t rx_id);
  /* implements data update callback */
  void UpdateData(const uint8_t data[]) override final;
  /* implements data printout */
  void PrintData() const override final;
  /* override base implementation with max current protection */
  void SetOutput(int16_t val) override final;

 private:
  volatile int16_t raw_current_get_ = 0;
};

/**
 * @brief DJI 3508 motor class
 */
class Motor3508 : public MotorCANBase {
 public:
  /* constructor wrapper over MotorCANBase */
  Motor3508(bsp::CAN* can, uint16_t rx_id);
  /* implements data update callback */
  void UpdateData(const uint8_t data[]) override final;
  /* implements data printout */
  void PrintData() const override final;
  /* override base implementation with max current protection */
  void SetOutput(int16_t val) override final;

 private:
  volatile int16_t raw_current_get_ = 0;
  volatile uint8_t raw_temperature_ = 0;
};

/**
 * @brief DJI 6020 motor class
 */
class Motor6020 : public MotorCANBase {
 public:
  /* constructor wrapper over MotorCANBase */
  Motor6020(bsp::CAN* can, uint16_t rx_id);
  /* implements data update callback */
  void UpdateData(const uint8_t data[]) override final;
  /* implements data printout */
  void PrintData() const override final;
  /* override base implementation with max current protection */
  void SetOutput(int16_t val) override final;

 private:
  volatile int16_t raw_current_get_ = 0;
  volatile int16_t raw_current_set_ = 0;
  volatile uint8_t raw_temperature_ = 0;

  static const int16_t CURRENT_CORRECTION = -1;  // current direction is reversed
};

/**
 * @brief DJI 6623 motor class
 */
class Motor6623 : public MotorCANBase {
 public:
  /* constructor wrapper over MotorCANBase */
  Motor6623(bsp::CAN* can, uint16_t rx_id);
  /* implements data update callback */
  void UpdateData(const uint8_t data[]) override final;
  /* implements data printout */
  void PrintData() const override final;
  /* override base implementation with max current protection */
  void SetOutput(int16_t val) override final;
  /* override default implementation with not implemented */
  float GetOmega() const override final;
  float GetOmegaDelta(const float target) const override final;

 private:
  volatile int16_t raw_current_get_ = 0;
  volatile int16_t raw_current_set_ = 0;

  static const int16_t CURRENT_CORRECTION = -1;  // current direction is reversed
};

/**
 * @brief PWM motor base class
 */
class MotorPWMBase : public MotorBase {
 public:
  /**
   * @brief constructor
   *
   * @param htim           HAL timer handle
   * @param channel        HAL timer channel, from [0, 4)
   * @param clock_freq     clock frequency associated with the timer, in [Hz]
   * @param output_freq    desired output frequency, in [Hz]
   * @param idle_throttle  idling pulse width, in [us]
   *
   * @note M3508 have idle_throttle about 1500, snail have idle_throttle about 1100
   */
  MotorPWMBase(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq,
               uint32_t idle_throttle);

  /**
   * @brief set and transmit output
   *
   * @param val offset value with respect to the idle throttle pulse width, in [us]
   */
  virtual void SetOutput(int16_t val) override;

 private:
  bsp::PWM pwm_;
  uint32_t idle_throttle_;
};

/**
 * @brief DJI snail 2305 motor class
 */
class Motor2305 : public MotorPWMBase {
 public:
  /* override base implementation with max current protection */
  void SetOutput(int16_t val) override final;
};

/**
 * @brief servomotor turning mode
 * @note the turning direction is determined as if user is facing the motor, may subject to
 *       change depending on motor type
 */
typedef enum {
  SERVO_CLOCKWISE = -1,   /* Servomotor always turn clockwisely                      */
  SERVO_NEAREST = 0,      /* Servomotor turn in direction that make movement minimum */
  SERVO_ANTICLOCKWISE = 1 /* Servomotor always turn anticlockwisely                  */
} servo_mode_t;

/**
 * @brief servomotor status
 * @note the turning direction is determined as if user is facing the motor, may subject to
 *       change depending on motor type
 */
typedef enum {
  TURNING_CLOCKWISE = -1,   /* Servomotor is turning clockwisely         */
  INPUT_REJECT = 0,         /* Servomotor rejecting current target input */
  TURNING_ANTICLOCKWISE = 1 /* Servomotor is turning anticlockwisely     */
} servo_status_t;

/**
 * @brief transmission ratios of DJI motors, reference to motor manuals for more details
 */
#define M3508P19_RATIO (3591.0 / 187) /* Transmission ratio of M3508P19 */
#define M2006P36_RATIO 36             /* Transmission ratio of M2006P36 */

typedef struct {
  servo_mode_t mode;  /* turning mode of servomotor, refer to type servo_mode_t                */
  servo_status_t dir; /* current turning direction of servomotor, refer to type servo_status_t */
  float speed;        /* motor shaft turning speed                                             */
} servo_jam_t;

class ServoMotor;  // declare first for jam_callback_t to have correct param type
/**
 * @brief jam callback template
 */
typedef void (*jam_callback_t)(ServoMotor* servo, const servo_jam_t data);

/**
 * @brief structure used when servomotor instance is initialized
 */
typedef struct {
  MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor      */
  servo_mode_t mode;        /* mode of turning, refer to type servo_mode_t       */
  float max_speed;          /* desired turning speed of motor shaft, in [rad/s]  */
  float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
  float transmission_ratio; /* transmission ratio of motor                       */
  float* omega_pid_param;   /* pid parameter used to control speed of motor      */
} servo_t;

/**
 * @brief wrapper class for motor to enable the motor shaft angle to be precisely controlled with
 *        possible external gearbox present
 * @note this is a calculation class that calculate the motor output for desired output, but it does
 * not directly command a motor to turn.
 */
class ServoMotor {
 public:
  /**
   * @brief base constructor
   *
   * @param servo         initialization struct, refer to type servo_t
   * @param proximity_in  critical difference angle for the motor to enter hold state
   * @param proximity_out critical difference angle for the motor to exit hold state
   *
   * @note proximity_out should be greater than proximity_in
   */
  ServoMotor(servo_t servo, float proximity_in = 0.05, float proximity_out = 0.15);

  /**
   * @brief set next target for servomotor, will have no effect if last set target has not been
   * achieved
   * @note if motor is not holding, call to this function will have no effect unless override is
   * true
   *
   * @param target   next target for the motor in [rad]
   * @param override if true, override current target even if motor is not holding right now
   * @return current turning mode of motor
   */
  servo_status_t SetTarget(const float target, bool override = false);

  /**
   * @brief set next target for servomotor, will have no effect if last set target has not been
   * achieved
   * @note if motor is not holding, call to this function will have no effect unless override is
   * true
   *
   * @param target   next target for the motor in [rad]
   * @param mode     servomotor turning mode override, will only have one-time effect
   * @param override if true, override current target even if motor is not holding right now
   * @return current turning mode of motor
   */
  servo_status_t SetTarget(const float target, const servo_mode_t mode, bool override = false);

  /**
   * @brief set turning speed of motor when moving
   *
   * @note should always be positive, negative inputs will be ignored
   *
   * @param max_speed speed of desired motor shaft turning speed, in [rad/s]
   */
  void SetMaxSpeed(const float max_speed);

  /**
   * @brief set acceleration of motor when moving
   *
   * @note should always be positive, negative inputs will be ignored
   *
   * @param max_acceleration speed of desired motor shaft turning speed, in [rad/s]
   */
  void SetMaxAcceleration(const float max_acceleration);

  /**
   * @brief calculate the output of the motors under current configuration
   * @note this function will not command the motor, it only calculate the desired input
   */
  void CalcOutput();

  /**
   * @brief if the motor is holding
   *
   * @return true  the motor is holding (i.e. not turning)
   * @return false the motor is not holding (i.e. turning)
   */
  bool Holding() const;

  /**
   * @brief get current servomotor target, in [rad]
   *
   * @return current target angle, range between [0, 2PI]
   */
  float GetTarget() const;

  /**
   * @brief register a callback function that would be called if motor is jammed
   * @note Jam detection uses a moving window across inputs to the motor. It uses a circular buffer
   * of size detect_period to store history inputs and calculates a rolling average of the inputs.
   *       Everytime the average of inputs is greater than
   *       effect_threshold * 32768(maximum command a motor can accept), the jam callback function
   * will be triggered once. The callback will only be triggered once each time the rolling average
   *       cross the threshold from lower to higher. For a standard jam callback function, refer to
   *       example motor_m3508_antijam
   *
   * @param callback         callback function to be registered
   * @param effort_threshold threshold for motor to be determined as jammed, ranged between (0, 1)
   * @param detect_period    detection window length
   */
  void RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                           uint8_t detect_period = 50);

  /**
   * @brief print out motor data
   */
  void PrintData() const;

  /**
   * @brief get motor angle, in [rad]
   *
   * @return radian angle, range between [0, 2PI]
   */
  float GetTheta() const;

  /**
   * @brief get angle difference (target - actual), in [rad]
   *
   * @param target  target angle, in [rad]
   *
   * @return angle difference, range between [-PI, PI]
   */
  float GetThetaDelta(const float target) const;

  /**
   * @brief get angular velocity, in [rad / s]
   *
   * @return angular velocity
   */
  float GetOmega() const;

  /**
   * @brief get angular velocity difference (target - actual), in [rad / s]
   *
   * @param target  target angular velocity, in [rad / s]
   *
   * @return difference angular velocity
   */
  float GetOmegaDelta(const float target) const;

  /**
   * @brief update the current theta for the servomotor
   * @note only used in CAN callback, do not call elsewhere
   *
   * @param data[]  raw data bytes
   */
  void UpdateData(const uint8_t data[]);

 private:
  // refer to servo_t for details
  MotorCANBase* motor_;
  servo_mode_t mode_;
  float max_speed_;
  float max_acceleration_;
  float transmission_ratio_;
  float proximity_in_;
  float proximity_out_;

  // angle control
  bool hold_;          /* true if motor is holding now, otherwise moving now                      */
  float target_;       /* desired target angle, range between [0, 2PI] in [rad]                   */
  float align_angle_;  /* motor angle when a instance of this class is created with that motor    */
  float motor_angle_;  /* current motor angle in [rad], with align_angle subtracted               */
  float offset_angle_; /* cumulative offset angle of motor shaft, range between [0, 2PI] in [rad] */
  float servo_angle_;  /* current angle of motor shaft, range between [0, 2PI] in [rad]           */
  servo_status_t dir_; /* current moving direction of motor (and motor shaft)                     */

  // jam detection
  jam_callback_t jam_callback_; /* callback function that will be invoked if motor jammed */
  int detect_head_;     /* circular buffer current head                                       */
  int detect_period_;   /* circular buffer length                                             */
  int detect_total_;    /* rolling sum of motor inputs                                        */
  int jam_threshold_;   /* threshold for rolling sum for the motor to be considered as jammed */
  int16_t* detect_buf_; /* circular buffer                                                    */

  // pid controllers
  PIDController omega_pid_; /* pid for controlling speed of motor */

  // edge detectors
  FloatEdgeDetector* wrap_detector_; /* detect motor motion across encoder boarder               */
  BoolEdgeDetector* hold_detector_;  /* detect motor is in mode toggling, reset pid accordingly  */
  BoolEdgeDetector* jam_detector_;   /* detect motor jam toggling, call jam callback accordingly */

  /**
   * @brief when motor is in SERVO_NEAREST mode, finding the nearest direction to make the turn
   *
   */
  void NearestModeSetDir_();

  /**
   * @brief set turning direction of the motor using specified turning mode
   *
   * @param mode mode of servomotor, refer to type servo_mode_t
   */
  void SetDirUsingMode_(servo_mode_t mode);
};

} /* namespace control */
