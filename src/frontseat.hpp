#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "pinout.h"                 // All physical pins for Teensy 4.1 frontseat
#include "micro_ros_platformio.h"   // micro-ROS over serial

#include <MS5837.h>
#include <Servo.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

#include <math.h>

void frontseat_setup();
void frontseat_loop();


void error_loop();
void attach_servos_and_arm();
void write_all_motors_from_cmds();
void write_all_motors_neutral();
void setup_depth_sensor();

void cb_thrust(const void * msgin);
void cb_estop(const void * msgin);
void cb_timer_fast(rcl_timer_t * timer, int64_t last_call_time);
void cb_timer_debug(rcl_timer_t * timer, int64_t last_call_time);
