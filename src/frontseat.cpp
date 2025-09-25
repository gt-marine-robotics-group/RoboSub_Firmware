/*
Frontseat firmware running on the Teensy4.1 on the RoboSub vehicle.

Communication with the main computer is done through MicroROS which allows
us to publish and subscribe to topics on the microcontroller as if 
this firmware was just another node in the ROS graph.

We handle the following in this file:
    - Motor controls to take the /thrust_cmds and send the correct signals to 
      the ESCs to drive the motors
        - Also publish /thrust_out as debug data
    - Read the E-Stop on /estop and stop motor commands when active
      backseat computer
    - Publish state of autonomy switch to /auto_enable
    - Publish depth sensor data to /depth_sensor

*/

// ------ Imports -------
#include "frontseat.hpp"

// ------ Constants -------
constexpr int SERIAL_BAUDRATE = 115200;

constexpr size_t NUM_THRUSTERS = 8;

constexpr int ESC_PWM_NEUTRAL_us = 1500;  // ESC neutral pulse width (microseconds)
constexpr int ESC_PWM_SCALE_us_per_percent = 4; // Microseconds added per 1%

constexpr int THROTTLE_MIN_PERCENT = -100; // Reverse 100% power
constexpr int THROTTLE_MAX_PERCENT =  100; // Forward 100% power

constexpr uint32_t ESC_ARM_DELAY_ms = 1500;  // Time to keep neutral for arming

constexpr uint32_t ERROR_BLINK_DELAY_ms = 10;
constexpr uint64_t EXECUTOR_SPIN_BUDGET_ns = RCL_MS_TO_NS(50);

constexpr uint32_t FAST_TIMER_PERIOD_ms = 100;
constexpr uint32_t SLOW_TIMER_PERIOD_ms = 200;

constexpr int AUTONOMY_DEBOUNCE_REQUIRED_TICKS = 5;

constexpr int FLUID_DENSITY_FRESHWATER_kg_per_m3 = 997;
constexpr float DEPTH_MIN_VALID_m = -10.0f; // Plausible minimum (for surface/bad reads)
constexpr float DEPTH_MAX_VALID_m = 200.0f; // Plausible maximum (tune to vehicle)
constexpr float DEPTH_MAGIC_BAD_READING_m = -346263.78125f; // Observed invalid value
constexpr uint32_t DEPTH_REINIT_BACKOFF_TICKS = 10; // Backoff between re-init attempts

#define DEPTH_SENSOR_MODEL MS5837::MS5837_30BA


// ------ Macros / Helpers ------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

// ESC Mapping
inline int clamp_int(int v, int lo, int hi) { 
  return (v < lo) ? lo : (v > hi) ? hi : v; 
}

inline int throttle_percent_to_pwm_us(int throttle_percent) {
  const int clamped_percent = clamp_int(throttle_percent, THROTTLE_MIN_PERCENT, THROTTLE_MAX_PERCENT);
  const int pwm_us = ESC_PWM_NEUTRAL_us + (ESC_PWM_SCALE_us_per_percent * clamped_percent);
  return pwm_us;
}

// ------ Hardware Objects ------
Servo motor_a, motor_b, motor_c, motor_d, motor_e, motor_f, motor_g, motor_h;
Servo* motors[NUM_THRUSTERS] = {
  &motor_a,
  &motor_b,
  &motor_c,
  &motor_d,
  &motor_e,
  &motor_f,
  &motor_g,
  &motor_h
};

MS5837 depth_sensor;


// ------ ROS Subscribers and Messages -------
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


// Subscriptions
rcl_subscription_t sub_thrust; // /thrust_cmds std_msgs/Float32MultiArray
rcl_subscription_t sub_estop;  // /estop stdd_msgs/Bool

std_msgs__msg__Float32MultiArray sub_thrust_msg; 
std_msgs__msg__Bool              sub_estop_msg;

// Publishers
rcl_publisher_t pub_depth;           // /depth_sensor std_msgs/Float32
rcl_publisher_t pub_autonomy_switch; // /auto_enable  std_msgs/Bool
rcl_publisher_t pub_thrustout;       // /thrust_out   std_msgs/Float32MultiArray


// Messages
std_msgs__msg__Float32 msg_depth;
std_msgs__msg__Bool msg_auto;
std_msgs__msg__Float32MultiArray msg_thrust_out;


// Timers
rcl_timer_t timer_fast;   // depth + auto
rcl_timer_t timer_debug;  // thrust_out


// ------- State -------
volatile bool e_stop = false;
volatile int  auto_enable_debounce_ticks = 0;

int thrust_cmd_percent[NUM_THRUSTERS] = {0};

bool depth_sensor_healthy = false;
uint32_t depth_reinit_backoff_counter = 0;


// ------ Error Loop ------
void error_loop() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  while (true) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(ERROR_BLINK_DELAY_ms);
  }
}

// ------ Hardware Helpers ------
void attach_motors_and_arm() {
  // Attach all ESCs
  for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
    motors[i]->attach(THRUSTER_PINS[i]);
  }

  // Arm/neutral
  write_all_motors_neutral();

  delay(ESC_ARM_DELAY_ms); // allow ESCs to recognize neutral
}

void write_all_motors_neutral() {
  for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
    motors[i]->writeMicroseconds(ESC_PWM_NEUTRAL_us);
  }
}

void write_all_motors_from_cmds() {
  for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
    const int pwm_us = throttle_percent_to_pwm_us(thrust_cmd_percent[i]);
    motors[i]->writeMicroseconds(pwm_us);
  }
}

void setup_depth_sensor() {
  Wire.setSDA(DEPTH_SENSOR_SDA_PIN);
  Wire.setSCL(DEPTH_SENSOR_SCL_PIN);
  depth_sensor_healthy = depth_sensor.init();

  if (depth_sensor_healthy) {
    depth_sensor.setModel(DEPTH_SENSOR_MODEL);
    depth_sensor.setFluidDensity(FLUID_DENSITY_FRESHWATER_kg_per_m3);
  }
}


// ------ Micro-ROS Callbacks ------
void cb_thrust(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg =
      (const std_msgs__msg__Float32MultiArray *)msgin; // cast msgin to the message type

  // Brief LED activity indicator
  digitalWrite(STATUS_LED_PIN, HIGH);

  const size_t n_in = (msg && msg->data.size > 0) ? msg->data.size : 0;
  for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
    const float v_in = (i < n_in) ? msg->data.data[i] : 0.0f;
    const int scaled_percent = (int)lroundf(v_in * 100.0f); // lroundf() rounds a float and returns a long
    thrust_cmd_percent[i] = clamp_int(scaled_percent, THROTTLE_MIN_PERCENT, THROTTLE_MAX_PERCENT);
  }

  digitalWrite(STATUS_LED_PIN, LOW);
}

void cb_estop(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (!msg) return;
  e_stop = msg->data;
}

void cb_timer_fast(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  // ---- AUTO ENABLE (debounced) ----
  const bool auto_switch_active_now = (digitalRead(AUTONOMY_SWITCH_PIN) == LOW); // active-low switch

  if (auto_switch_active_now) {
    if (auto_enable_debounce_ticks < AUTONOMY_DEBOUNCE_REQUIRED_TICKS) {
      auto_enable_debounce_ticks++;
    }
  } else {
    auto_enable_debounce_ticks = 0;
  }

  const bool auto_enable_publish = (auto_enable_debounce_ticks >= AUTONOMY_DEBOUNCE_REQUIRED_TICKS);
  msg_auto.data = auto_enable_publish;
  RCSOFTCHECK(rcl_publish(&pub_autonomy_switch, &msg_auto, nullptr));

  // ---- DEPTH ----
  float out_depth_m = -1.0f;

  if (!depth_sensor_healthy) {
    if ((depth_reinit_backoff_counter++ % DEPTH_REINIT_BACKOFF_TICKS) == 0) {
      depth_sensor_healthy = depth_sensor.init();
      if (depth_sensor_healthy) {
        depth_sensor.setModel(DEPTH_SENSOR_MODEL);
        depth_sensor.setFluidDensity(FLUID_DENSITY_FRESHWATER_kg_per_m3);
      }
    }
  }

  if (depth_sensor_healthy) {
    depth_sensor.read();
    const float depth_m = depth_sensor.depth();

    const bool depth_ok =
      isfinite(depth_m) &&
      (depth_m != DEPTH_MAGIC_BAD_READING_m) &&
      (depth_m > DEPTH_MIN_VALID_m) &&
      (depth_m < DEPTH_MAX_VALID_m);

    if (depth_ok) {
      out_depth_m = depth_m;
    } else {
      depth_sensor_healthy = false;
      out_depth_m = -2.0f;
    }
  }

  msg_depth.data = out_depth_m;
  RCSOFTCHECK(rcl_publish(&pub_depth, &msg_depth, nullptr));
}

void cb_timer_debug(rcl_timer_t * timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;  // Avoid compiler complaining

  for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
    msg_thrust_out.data.data[i] = static_cast<float>(thrust_cmd_percent[i]);
  }
  RCSOFTCHECK(rcl_publish(&pub_thrustout, &msg_thrust_out, nullptr));
}


// ------ Setup ------
void setup() {
  // loop_time = millis();

  delay(100);

  Serial.begin(SERIAL_BAUDRATE);
  set_microros_serial_transports(Serial);


  // Genric I/O
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  pinMode(AUTONOMY_SWITCH_PIN, INPUT_PULLUP);

  // Motors
  attach_motors_and_arm();

  setup_depth_sensor();

  // microROS allocator 
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Node
  RCCHECK(rclc_node_init_default(&node, "frontseat_teensy41", "", &support));


  // --- Subscribers ---
  RCCHECK(rclc_subscription_init_default(
    &sub_thrust,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/thrust_cmds"));

  RCCHECK(rclc_subscription_init_default(
    &sub_estop,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/estop"));

  // --- Publishers ---
  RCCHECK(rclc_publisher_init_default(
    &pub_depth,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/depth_sensor"));

  RCCHECK(rclc_publisher_init_default(
    &pub_autonomy_switch,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/auto_enable"));

  RCCHECK(rclc_publisher_init_default(
    &pub_thrustout,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/thrust_out"));


  static float thrust_out_buf[NUM_THRUSTERS] = {0.0f};
  msg_thrust_out.data.data = thrust_out_buf;
  msg_thrust_out.data.size = NUM_THRUSTERS;
  msg_thrust_out.data.capacity = NUM_THRUSTERS;

  // --- Timers ---
  RCCHECK(rclc_timer_init_default(
    &timer_fast,
    &support,
    RCL_MS_TO_NS(FAST_TIMER_PERIOD_ms),
    cb_timer_fast));

  RCCHECK(rclc_timer_init_default(
    &timer_debug,
    &support,
    RCL_MS_TO_NS(SLOW_TIMER_PERIOD_ms),
    cb_timer_debug));

  // --- Executor ---
  // 2 subscriptions + 2 timers = 4 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &sub_thrust, &sub_thrust_msg, cb_thrust,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_estop,  &sub_estop_msg,  cb_estop,   ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_fast));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_debug));
}


void loop() {
  // Run executor work
  RCCHECK(rclc_executor_spin_some(&executor, EXECUTOR_SPIN_BUDGET_ns));

  if (depth_sensor_healthy) {
    depth_sensor.read();
  } else {
    if (depth_sensor.init()) {
      depth_sensor_healthy = true;
      depth_sensor.setModel(DEPTH_SENSOR_MODEL);
      depth_sensor.setFluidDensity(FLUID_DENSITY_FRESHWATER_kg_per_m3);
      depth_sensor.read();

    }
  }

  // Drive ESCs from current thrust commands, honoring e-stop
  if (e_stop) {
    write_all_motors_neutral();
  } else {
    write_all_motors_from_cmds();
  }
}
