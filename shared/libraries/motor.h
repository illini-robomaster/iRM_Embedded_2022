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
   * @return radian angle
   */
  virtual float GetTheta() const;

  /**
   * @brief get angle difference (target - actual), in [rad]
   *
   * @param target  target angle, in [rad]
   *
   * @return angle difference
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
 * @enum Servomotor turning mode
 */
typedef enum {
  SERVO_CLOCKWISE       = -1,   /* Servomotor always turn clockwisely                      */
  SERVO_NEAREST         =  0,   /* Servomotor turn in direction that make movement minimum */
  SERVO_ANTICLOCKWISE   =  1    /* Servomotor always turn anticlockwisely                  */
} servo_mode_t;

/**
 * @brief servomotor turning mode
 */
typedef enum {
  TURNING_CLOCKWISE     = -1,   /* Servomotor is turning clockwisely         */
  INPUT_REJECT          =  0,   /* Servomotor rejecting current target input */
  TURNING_ANTICLOCKWISE =  1    /* Servomotor is turning anticlockwisely     */
} servo_status_t;

/**
 * @brief transmission ratios of DJI motors, reference to motor manuals for more details
 */
#define M3508P19_RATIO (3591.0 / 187) /* Transmission ratio of M3508P19 */
#define M2006P36_RATIO 36             /* Transmission ratio of M2006P36 */

/**
 * @brief structure used when servomotor instance is initialized
 */
typedef struct {
  MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor */
  servo_mode_t mode;        /* mode of turning, refer to type servo_mode_t  */
  float speed;              /* desired turning speed of motor, in [rad/s]   */
  float transmission_ratio; /* transmission ratio of motor                  */
  float move_Kp;            /* Kp of pid that used to control omega         */
  float move_Ki;            /* Ki of pid that used to control omega         */
  float move_Kd;            /* Kd of pid that used to control omega         */
  float hold_Kp;            /* Kp of pid that used to control theta         */
  float hold_Ki;            /* Ki of pid that used to control theta         */
  float hold_Kd;            /* Kd of pid that used to control theta         */
} servo_t;

/**
 * @brief wrapper class for motor to enable it to be controlled as a servomotor
 * @note the sampling frequency (the frequency of calling CalcOutput) should be high enough (refer to
 *       Nyquist sampling frequency) and input PID should not be to "hard", otherwise the calculations 
 *       could be wrong.
 */
class ServoMotor {
  public:
    /**
     * @brief base constructor
     *
     * @param servo     initialization struct, refer to type servo_t
     * @param proximity critical difference angle for the motor to stop turining when approaching target
     */
    ServoMotor(servo_t servo, float proximity = 0.1);

    /**
     * @brief set next target for servomotor, will have no effect if last set target has not been achieved
     * 
     * @param target next target for the motor in [rad]
     * @return int   current turning mode of motor, refer to type servo_status_t
     */
    int SetTarget(const float target, bool override = false);

    /**
     * @brief set next target for servomotor, will have no effect if last set target has not been achieved
     * 
     * @param target next target for the motor in [rad]
     * @param mode   servomotor turning mode override, will only have one-time effect
     * @return int   current turning mode of motor, refer to type servo_status_t
     */
    int SetTarget(const float target, const servo_mode_t mode, bool override = false);

    /**
     * @brief set turning speed of motor when moving, should always be positive
     * 
     * @param speed speed of desired turning speed, in [rad/s]
     */
    void SetSpeed(const float speed);

    /**
     * @brief calculate the output of the motors under current configuration
     * @note should have high calling frequency to ensure best result
     * @note this function will not transmit output to motor, it only calculate the desired input
     */
    void CalcOutput();

    /**
     * @brief if the motor is holding
     * 
     * @return true  the motor is in holding state (i.e. not turning)
     * @return false the motor is not holding (i.e. turning)
     */
    bool Holding() const;

    /**
     * @brief print out motor data
     */
    void PrintData() const;

    /**
     * @brief get motor angle, in [rad]
     *
     * @return radian angle
     */
    float GetTheta() const;

    /**
     * @brief get angle difference (target - actual), in [rad]
     *
     * @param target  target angle, in [rad]
     *
     * @return angle difference
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

  private:
    MotorCANBase* motor_;
    servo_mode_t mode_;
    float speed_;
    float transmission_ratio_;
    float proximity_;

    bool hold_;
    float target_;
    float align_angle_;
    float motor_angle_;
    float offset_angle_;
    float servo_angle_;
    int dir_;    

    PIDController move_pid_;
    PIDController hold_pid_;

    FloatEdgeDetector* wrap_detector_;
    BoolEdgeDetector* hold_detector_;

    /**
     * @brief update the current theta for the servomotor
     */
    void AngleUpdate_();

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
