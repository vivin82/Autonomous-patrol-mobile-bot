#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>

// ═══════════════════════════════════════════════════
// WIFI CREDENTIALS
// ═══════════════════════════════════════════════════
const char* SSID     = "xxx";
const char* PASSWORD = "xxxx";

// ═══════════════════════════════════════════════════
// ROS MASTER IP (Your Laptop)
// ═══════════════════════════════════════════════════
IPAddress ROS_MASTER_IP(xx,xxx,xxx,xxx);
const uint16_t ROS_PORT = 11411;

// ═══════════════════════════════════════════════════
// ROBOT PARAMETERS
// ═══════════════════════════════════════════════════
const float TRACK_WIDTH = 0.38;
const float MAX_VEL     = 0.4;

// ═══════════════════════════════════════════════════
// L298N #1 — LEFT wheels
// ═══════════════════════════════════════════════════
#define L_ENA  27
#define L_IN1  26
#define L_IN2  25
#define L_IN3  33
#define L_IN4  32
#define L_ENB  14

// ═══════════════════════════════════════════════════
// L298N #2 — RIGHT wheels
// ═══════════════════════════════════════════════════
#define R_ENA  19
#define R_IN1  18
#define R_IN2   5
#define R_IN3  17
#define R_IN4  16
#define R_ENB   4

// ═══════════════════════════════════════════════════
// MPU-6050 I2C
// ═══════════════════════════════════════════════════
#define IMU_SDA 21
#define IMU_SCL 22

// ═══════════════════════════════════════════════════
// GLOBALS & TIMERS
// ═══════════════════════════════════════════════════
MPU6050 mpu;
int16_t ax_off = 0, ay_off = 0, az_off = 0;
int16_t gx_off = 0, gy_off = 0, gz_off = 0;

unsigned long last_cmd_ms      = 0;
unsigned long last_imu_ms      = 0;
unsigned long last_wifi_check  = 0;

const unsigned long WATCHDOG_MS     = 1000;
const unsigned long IMU_INTERVAL_MS = 50;
const unsigned long WIFI_CHECK_MS   = 5000;

// ═══════════════════════════════════════════════════
// 100% POWER MOTOR CONTROL LOGIC (NO PWM)
// ═══════════════════════════════════════════════════
void setMotorPair(int en_pin_a, int in1, int in2,
                  int en_pin_b, int in3, int in4,
                  float speed) {
  if (speed > 0.1f) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    digitalWrite(en_pin_a, HIGH);
    digitalWrite(en_pin_b, HIGH);
  }
  else if (speed < -0.1f) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
    digitalWrite(en_pin_a, HIGH);
    digitalWrite(en_pin_b, HIGH);
  }
  else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, LOW);
    digitalWrite(en_pin_a, LOW);
    digitalWrite(en_pin_b, LOW);
  }
}

void stopAll() {
  setMotorPair(L_ENA, L_IN1, L_IN2, L_ENB, L_IN3, L_IN4, 0.0f);
  setMotorPair(R_ENA, R_IN1, R_IN2, R_ENB, R_IN3, R_IN4, 0.0f);
}

// ═══════════════════════════════════════════════════
// CMD_VEL CALLBACK — WITH TURN FIX
// ═══════════════════════════════════════════════════
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  last_cmd_ms = millis();

  float v = -msg.linear.x;
  float w = msg.angular.z;

  float left  = (v - w * TRACK_WIDTH / 2.0f);
  float right = (v + w * TRACK_WIDTH / 2.0f);

  // Normalize so the dominant side always reaches ±1.0
  float max_val = max(abs(left), abs(right));
  if (max_val > 1.0f) {
    left  /= max_val;
    right /= max_val;
  }

  // Pure turn boost: if barely moving forward, use full power in opposite directions
  if (abs(v) < 0.05f && abs(w) > 0.05f) {
    left  = (w < 0) ?  1.0f : -1.0f;
    right = (w < 0) ? -1.0f :  1.0f;
  }

  setMotorPair(L_ENA, L_IN1, L_IN2, L_ENB, L_IN3, L_IN4, left);
  setMotorPair(R_ENA, R_IN1, R_IN2, R_ENB, R_IN3, R_IN4, right);
}

// ═══════════════════════════════════════════════════
// ROS OBJECTS
// ═══════════════════════════════════════════════════
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher  imu_pub("/imu/data_raw", &imu_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmdVelCallback);

// ═══════════════════════════════════════════════════
// WIFI CONNECT
// ═══════════════════════════════════════════════════
void connectWifi() {
  Serial.println("\n--------------------------------");
  Serial.print("Attempting to connect to WiFi SSID: ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nSUCCESS: Connected to WiFi!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Attempting to connect to ROS Master at: ");
    Serial.println(ROS_MASTER_IP);
    Serial.println("--------------------------------\n");
  } else {
    Serial.println("\nERROR: Failed to connect to WiFi.");
  }
}

// ═══════════════════════════════════════════════════
// IMU CALIBRATION
// ═══════════════════════════════════════════════════
void calibrateMPU() {
  nh.loginfo("IMU calibrating — keep robot still...");
  Serial.println("IMU calibrating — keep robot completely still...");

  const int SAMPLES = 500;
  long ax_s=0, ay_s=0, az_s=0;
  long gx_s=0, gy_s=0, gz_s=0;

  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_s += ax;  ay_s += ay;  az_s += az;
    gx_s += gx;  gy_s += gy;  gz_s += gz;
    delay(3);
  }

  ax_off = ax_s / SAMPLES;
  ay_off = ay_s / SAMPLES;
  az_off = az_s / SAMPLES - 16384;
  gx_off = gx_s / SAMPLES;
  gy_off = gy_s / SAMPLES;
  gz_off = gz_s / SAMPLES;

  nh.loginfo("IMU calibration done.");
  Serial.println("IMU calibration complete. Robot is ready.");
}

// ═══════════════════════════════════════════════════
// IMU PUBLISH
// ═══════════════════════════════════════════════════
void publishImu() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  ax -= ax_off;  ay -= ay_off;  az -= az_off;
  gx -= gx_off;  gy -= gy_off;  gz -= gz_off;

  const float ACCEL_SCALE = 9.80665f / 16384.0f;
  const float GYRO_SCALE  = (M_PI / 180.0f) / 131.0f;

  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.orientation_covariance[0] = -1;

  imu_msg.angular_velocity.x = gx * GYRO_SCALE;
  imu_msg.angular_velocity.y = gy * GYRO_SCALE;
  imu_msg.angular_velocity.z = gz * GYRO_SCALE;

  float gv = pow(0.05f * M_PI / 180.0f, 2);
  imu_msg.angular_velocity_covariance[0] = gv;
  imu_msg.angular_velocity_covariance[4] = gv;
  imu_msg.angular_velocity_covariance[8] = gv;

  imu_msg.linear_acceleration.x = ax * ACCEL_SCALE;
  imu_msg.linear_acceleration.y = ay * ACCEL_SCALE;
  imu_msg.linear_acceleration.z = az * ACCEL_SCALE;

  float av = pow(0.01f, 2);
  imu_msg.linear_acceleration_covariance[0] = av;
  imu_msg.linear_acceleration_covariance[4] = av;
  imu_msg.linear_acceleration_covariance[8] = av;

  imu_pub.publish(&imu_msg);
}

// ═══════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);

  int motorPins[] = {L_IN1, L_IN2, L_IN3, L_IN4, R_IN1, R_IN2, R_IN3, R_IN4,
                     L_ENA, L_ENB, R_ENA, R_ENB};
  for (int pin : motorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  stopAll();

  Wire.begin(IMU_SDA, IMU_SCL);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  connectWifi();

  nh.getHardware()->setConnection(ROS_MASTER_IP, ROS_PORT);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(imu_pub);

  Serial.println("Waiting for ROS Master connection...");
  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
  Serial.println("ROS Master Connected!");

  calibrateMPU();
}

// ═══════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════
void loop() {
  if (millis() - last_wifi_check > WIFI_CHECK_MS) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi dropped. Reconnecting...");
      connectWifi();
    }
    last_wifi_check = millis();
  }

  nh.spinOnce();

  if (millis() - last_cmd_ms > WATCHDOG_MS) {
    stopAll();
  }

  if (millis() - last_imu_ms >= IMU_INTERVAL_MS) {
    publishImu();
    last_imu_ms = millis();
  }

  delay(1);
}
