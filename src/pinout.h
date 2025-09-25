#ifndef FRONTSEAT_PINOUT_H
#define FRONTSEAT_PINOUT_H

/*
  Target MCU: Teensy 4.1

  Notes:
  - All motor outputs are standard Servo/ESC PWM via the Arduino Servo library.
  - Depth sensor (MS5837) is on the Teensy 4.1 secondary I2C bus (Wire2: SDA2=25, SCL2=24).
*/
#include <Arduino.h>

// ------ Motor Pins ------
constexpr uint8_t A_SIG_PIN = 33; // BACK LEFT  VERT  (motor_a)
constexpr uint8_t B_SIG_PIN = 34; // BACK RIGHT VERT  (motor_b)
constexpr uint8_t C_SIG_PIN = 35; // FRONT LEFT VERT  (motor_c)
constexpr uint8_t D_SIG_PIN = 36; // FRONT RIGHT VERT (motor_d)
constexpr uint8_t E_SIG_PIN = 37; // BACK LEFT  HORIZ (motor_e)
constexpr uint8_t F_SIG_PIN = 39; // BACK RIGHT HORIZ (motor_f)
constexpr uint8_t G_SIG_PIN = 40; // FRONT LEFT HORIZ (motor_g)
constexpr uint8_t H_SIG_PIN = 41; // FRONT RIGHT HORIZ (motor_h)

constexpr uint8_t THRUSTER_PINS[8] = {
  A_SIG_PIN, B_SIG_PIN, C_SIG_PIN, D_SIG_PIN, E_SIG_PIN, F_SIG_PIN, G_SIG_PIN, H_SIG_PIN
};

// ------ Autonomy Switch ------
constexpr uint8_t AUTONOMY_SWITCH_PIN = 10;  

// ------ Depth Sensor Pins -------
constexpr uint8_t DEPTH_SENSOR_SDA_PIN = 25; // SDA2
constexpr uint8_t DEPTH_SENSOR_SCL_PIN = 24; // SCL2
constexpr uint8_t MS5837_I2C_ADDR = 0x76;

// ------ Generic I/O -------
constexpr uint8_t STATUS_LED_PIN  = 13; // Onboard LED 

#endif 