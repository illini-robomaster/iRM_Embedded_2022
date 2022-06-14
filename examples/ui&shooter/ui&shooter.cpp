//
// Created by Jerry Wang on 2022/6/12.
//
#include "main.h"

#include "bsp_gpio.h"
#include "bsp_laser.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "shooter.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

bsp::CAN* can = nullptr;
control::MotorCANBase* left_flywheel_motor = nullptr;
control::MotorCANBase* right_flywheel_motor = nullptr;
control::MotorCANBase* load_motor = nullptr;

remote::DBUS* dbus = nullptr;
control::ServoMotor* load_servo = nullptr;
control::Shooter* shooter = nullptr;

BoolEdgeDetector key_detector(false);

void RM_RTOS_INIT(){
  dbus = new remote::DBUS(&huart1);

  can = new bsp::CAN(&hcan1, 0x201);
  left_flywheel_motor = new control::Motor3508(can, 0x201);
  right_flywheel_motor = new control::Motor3508(can, 0x202);
  load_motor = new control::Motor3508(can, 0x203);

  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = left_flywheel_motor;
  shooter_data.right_flywheel_motor = right_flywheel_motor;
  shooter_data.load_motor = load_motor;
  shooter = new control::Shooter(shooter_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[] = {left_flywheel_motor, right_flywheel_motor, load_motor};
  bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);

  while (true) {
    key_detector.input(key.Read());
    if (key_detector.posEdge()){
        shooter->SetFlywheelSpeed(450);
        shooter->LoadNext();
    }

    shooter->SetFlywheelSpeed(dbus->ch1);
    if (dbus->ch3 > 500) shooter->LoadNext();
    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors, 3);
    osDelay(10);
  }
}