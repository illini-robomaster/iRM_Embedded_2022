#pragma once

#include "bsp_uart.h"

namespace remote {

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t l;
  uint8_t r;
} __packed mouse_t;

typedef union {
  uint16_t code;
  struct {
    uint16_t W : 1;
    uint16_t S : 1;
    uint16_t A : 1;
    uint16_t D : 1;
    uint16_t SHIFT : 1;
    uint16_t CTRL : 1;
    uint16_t Q : 1;
    uint16_t E : 1;
    uint16_t R : 1;
    uint16_t F : 1;
    uint16_t G : 1;
    uint16_t Z : 1;
    uint16_t X : 1;
    uint16_t C : 1;
    uint16_t V : 1;
    uint16_t B : 1;
  } __packed bit;
} __packed keyboard_t;

typedef enum {
  UP = 1,
  DOWN = 2,
  MID = 3,
} switch_t;

class DBUS : public bsp::UART {
 public:
  // intialize DBUS the same way as a generic UART peripheral
  DBUS(UART_HandleTypeDef* huart);

  // Add custom rx data handler
  void RxCompleteCallback() override final;

  // rocker channel information
  int16_t ch0;  // S1*             *S2
  int16_t ch1;  //   C3-^       ^-C1
  int16_t ch2;  // C2-<   >+ -<   >+C0
  int16_t ch3;  //     +v       v+
  // left and right switch information
  switch_t swl;
  switch_t swr;
  // mouse movement and button information
  mouse_t mouse;
  // keyboard key information
  keyboard_t keyboard;
  // timestamp of the update interrupt
  uint32_t timestamp;

  static const int16_t ROCKER_MIN = -660;
  static const int16_t ROCKER_MAX = 660;
};

} /* namespace remote */
