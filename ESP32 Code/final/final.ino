#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

// ================= MOTOR PINS =================
#define RIGHT_DIR   12
#define RIGHT_PWM   13
#define LEFT_DIR    27
#define LEFT_PWM    14
#define JACK_RPWM   33
#define JACK_LPWM   25
#define BTS_EN      26

// ================= ULTRASONIC =================
#define TRIG_FRONT  15
#define ECHO_FRONT  2
#define TRIG_LEFT   32
#define ECHO_LEFT   35
#define TRIG_RIGHT  17
#define ECHO_RIGHT  5

// ================= ROS OBJECTS =================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_publisher_t imu_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t ultrasonic_pub;
rcl_publisher_t tf_pub;
rcl_subscription_t motor_sub;

// ================= MESSAGES =================
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32MultiArray ultrasonic_msg;
std_msgs__msg__Int32MultiArray motor_msg;
geometry_msgs__msg__TransformStamped tf_msg;

// ================= IMU =================
MPU6050 mpu;
Quaternion q;
VectorInt16 aa, gg;
VectorFloat gravity;
uint8_t fifoBuffer[64];
bool dmpReady = false;

// ================= ODOM =================
float x = 0, y = 0, theta = 0;
unsigned long last_time = 0;
int last_L = 0, last_R = 0;

const float wheel_base = 0.20;
const float motor_scale = 0.002;

// ================= TIME SYNC =================
bool time_synced = false;
unsigned long last_sync_attempt = 0;
const unsigned long SYNC_RETRY_INTERVAL = 5000; // Try every 5 seconds
int sync_attempts = 0;

// ================= HELPERS =================
long readUltrasonic(int t, int e) {
  digitalWrite(t, LOW); delayMicroseconds(2);
  digitalWrite(t, HIGH); delayMicroseconds(10);
  digitalWrite(t, LOW);
  long d = pulseIn(e, HIGH, 30000);
  return d * 0.034 / 2;
}

#define RCCHECK(fn) if ((fn) != RCL_RET_OK) while (1)
#define RCSOFTCHECK(fn) (void)(fn)

// ================= TIME SYNC FUNCTION =================
bool sync_time() {
  // Attempt time synchronization
  rmw_ret_t ret = rmw_uros_sync_session(1000);
  
  if (ret == RMW_RET_OK) {
    delay(100); // Give time for sync to settle
    
    // Verify sync worked
    int64_t now_ns = rmw_uros_epoch_nanos();
    
    // Unix epoch seconds should be > 1,000,000,000 (Sep 2001)
    // Current time should be > 1,700,000,000 (Nov 2023)
    if (now_ns > 1700000000000000000LL) {
      return true;
    }
  }
  
  return false;
}

// ================= MOTOR =================
void motorDrive(int L, int R, int J) {
  digitalWrite(LEFT_DIR,  L >= 0);
  digitalWrite(RIGHT_DIR, R >= 0);
  ledcWrite(0, abs(L));
  ledcWrite(1, abs(R));
  digitalWrite(BTS_EN, HIGH);

  if (J > 0) { ledcWrite(2, abs(J)); ledcWrite(3, 0); }
  else if (J < 0) { ledcWrite(2, 0); ledcWrite(3, abs(J)); }
  else { ledcWrite(2, 0); ledcWrite(3, 0); }
}

// ================= MOTOR CALLBACK =================
void motor_cb(const void *msg) {
  auto *m = (std_msgs__msg__Int32MultiArray *)msg;
  if (m->data.size < 3) return;
  last_L = m->data.data[0];
  last_R = m->data.data[1];
  motorDrive(last_L, last_R, m->data.data[2]);
}

// ================= TIMER =================
void timer_cb(rcl_timer_t *, int64_t) {
  // ================= TIME SYNC CHECK =================
  if (!time_synced) {
    unsigned long now = millis();
    if (now - last_sync_attempt > SYNC_RETRY_INTERVAL) {
      last_sync_attempt = now;
      sync_attempts++;
      
      if (sync_time()) {
        time_synced = true;
      }
    }
    
    // Don't publish if not synced
    return;
  }
  
  // ================= GET TIMESTAMP =================
  int64_t now_ns = rmw_uros_epoch_nanos();
  
  // Double-check time is still valid
  if (now_ns < 1700000000000000000LL) {
    time_synced = false; // Lost sync
    return;
  }
  
  int32_t sec = (int32_t)(now_ns / 1000000000LL);
  uint32_t nanosec = (uint32_t)(now_ns % 1000000000LL);

  // ================= ODOMETRY CALCULATION =================
  unsigned long now_ms = millis();
  float dt = (now_ms - last_time) / 1000.0;
  last_time = now_ms;

  float vL = last_L * motor_scale;
  float vR = last_R * motor_scale;
  float v  = (vL + vR) / 2.0;
  float w  = (vR - vL) / wheel_base;

  theta += w * dt;
  
  // Normalize theta to [-p, p]
  while(theta > M_PI) theta -= 2.0 * M_PI;
  while(theta < -M_PI) theta += 2.0 * M_PI;
  
  x += v * cos(theta) * dt;
  y += v * sin(theta) * dt;

  // ================= PUBLISH ODOMETRY =================
  odom_msg.header.stamp.sec = sec;
  odom_msg.header.stamp.nanosec = nanosec;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = w;

  RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));

  // ================= PUBLISH TF: odom -> base_link =================
  tf_msg.header.stamp.sec = sec;
  tf_msg.header.stamp.nanosec = nanosec;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = sin(theta / 2.0);
  tf_msg.transform.rotation.w = cos(theta / 2.0);

  RCSOFTCHECK(rcl_publish(&tf_pub, &tf_msg, NULL));

  // ================= PUBLISH IMU =================
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gg, fifoBuffer);

    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;
    imu_msg.orientation.w = q.w;

    imu_msg.angular_velocity.x = gg.x * (M_PI / 180.0) / 131.0;
    imu_msg.angular_velocity.y = gg.y * (M_PI / 180.0) / 131.0;
    imu_msg.angular_velocity.z = gg.z * (M_PI / 180.0) / 131.0;
    
    imu_msg.linear_acceleration.x = aa.x / 16384.0 * 9.81;
    imu_msg.linear_acceleration.y = aa.y / 16384.0 * 9.81;
    imu_msg.linear_acceleration.z = aa.z / 16384.0 * 9.81;
    
    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nanosec;

    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  }

  // ================= PUBLISH ULTRASONIC =================
  ultrasonic_msg.data.data[0] = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  ultrasonic_msg.data.data[1] = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  ultrasonic_msg.data.data[2] = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  RCSOFTCHECK(rcl_publish(&ultrasonic_pub, &ultrasonic_msg, NULL));
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  
  set_microros_transports();
  
  Serial.println("Waiting for micro-ROS agent...");
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_robot", "", &support));

  Serial.println("Node initialized, attempting time sync...");
  
  // ================= CRITICAL: SYNCHRONIZE TIME =================
  // Try multiple times to sync
  for(int i = 0; i < 10; i++) {
    Serial.print("Time sync attempt ");
    Serial.print(i + 1);
    Serial.print("/10... ");
    
    if (sync_time()) {
      time_synced = true;
      Serial.println("SUCCESS!");
      
      int64_t now_ns = rmw_uros_epoch_nanos();
      int32_t sec = (int32_t)(now_ns / 1000000000LL);
      Serial.print("Current time (seconds): ");
      Serial.println(sec);
      break;
    } else {
      Serial.println("FAILED");
      delay(500);
    }
  }
  
  if (!time_synced) {
    Serial.println("WARNING: Time sync failed! Will retry during operation.");
  }
  
  // ================= INITIALIZE IMU =================
  Wire.begin();
  mpu.initialize();
  dmpReady = (mpu.dmpInitialize() == 0);
  mpu.setDMPEnabled(true);

  // ================= INITIALIZE MOTORS =================
  pinMode(LEFT_DIR, OUTPUT); 
  pinMode(RIGHT_DIR, OUTPUT); 
  pinMode(BTS_EN, OUTPUT);
  
  ledcSetup(0, 20000, 8); ledcAttachPin(LEFT_PWM, 0);
  ledcSetup(1, 20000, 8); ledcAttachPin(RIGHT_PWM, 1);
  ledcSetup(2, 20000, 8); ledcAttachPin(JACK_RPWM, 2);
  ledcSetup(3, 20000, 8); ledcAttachPin(JACK_LPWM, 3);

  // ================= INITIALIZE ULTRASONIC =================
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // ================= CREATE PUBLISHERS & SUBSCRIBERS =================
  RCCHECK(rclc_publisher_init_default(&imu_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/awmr/imu"));
  RCCHECK(rclc_publisher_init_default(&odom_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"));
  RCCHECK(rclc_publisher_init_default(&ultrasonic_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/awmr/ultrasonic"));
  RCCHECK(rclc_publisher_init_default(&tf_pub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped), "/tf"));
  RCCHECK(rclc_subscription_init_default(&motor_sub, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/awmr/cmd_motor"));

  // ================= ALLOCATE MESSAGE MEMORY =================
  ultrasonic_msg.data.capacity = 3;
  ultrasonic_msg.data.size = 3;
  ultrasonic_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));

  motor_msg.data.capacity = 3;
  motor_msg.data.size = 3;
  motor_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));

  // ================= CREATE TIMER & EXECUTOR =================
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_cb));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, motor_cb, ON_NEW_DATA));

  // ================= SET FRAME IDs =================
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  rosidl_runtime_c__String__assign(&tf_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&tf_msg.child_frame_id, "base_link");

  // ================= SET COVARIANCES =================
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
    if (i < 9) {
      imu_msg.orientation_covariance[i] = 0.0;
      imu_msg.angular_velocity_covariance[i] = 0.0;
      imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
  }

  odom_msg.pose.covariance[0]  = 0.05;
  odom_msg.pose.covariance[7]  = 0.05;
  odom_msg.pose.covariance[35] = 0.1;

  odom_msg.twist.covariance[0]  = 0.1;
  odom_msg.twist.covariance[35] = 0.2;

  imu_msg.orientation_covariance[0] = 0.01;
  imu_msg.orientation_covariance[4] = 0.01;
  imu_msg.orientation_covariance[8] = 0.01;

  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  last_time = millis();
  
  Serial.println("Setup complete!");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}