#include "Keyboard.h"
#include <MsTimer2.h>
#include<Wire.h>
#include <math.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050

#define HANDLE_PIN A0
#define SPACE_PIN 16
#define BRAKE_PIN 10
#define CALIBRATION_PIN 14

#define N_LED 5
const int led_pins[2][N_LED] = {
  {A3, A1, A0, 15, A2},
  {6, 4, 5, 8, 9} // left
};

#ifndef KEY_SPACE
#define KEY_SPACE 0x20
#endif

// Quaternion and rotation parameters
#define UNSENSE_HANDLE_VAL 2
#define GYRO_SCALE 16.4    // ±2000deg/s scale factor
#define ACCEL_SCALE 16384.0 // ±2g scale factor
#define FILTER_ALPHA 0.98  // Complementary filter coefficient
#define DT 0.025           // 25ms sampling period

// Quaternion structure
struct Quaternion {
  float w, x, y, z;
};

// Global variables for quaternion-based rotation
Quaternion q = {1.0, 0.0, 0.0, 0.0};  // Initial quaternion (identity)
Quaternion q_initial = {1.0, 0.0, 0.0, 0.0};  // Initial orientation for calibration
float handle_angle = 0.0;  // Current handle angle in radians
bool calibration_done = false;

#define HANDLE_NEUTRAL 477
int handle_val = 0;
bool handle_pushed = false;
int handle_n_pushed = 0;
int handle_n_released = 0;
#define RIGHT 0
#define LEFT 1
bool pushed[2] = {false, false};

#define N_DUTY_RATIO 18

const int duty_ratio_arr[N_DUTY_RATIO][2] = {
  { 1, 12 },
  { 1, 10 },
  { 1, 8 },
  { 1, 4 },
  { 1, 2 },
  { 1, 1 },
  { 2, 1 },
  { 3, 1 },
  { 4, 1 },
  { 5, 1 },
  { 6, 1 },
  { 7, 1 },
  { 8, 1 },
  { 9, 1 },
  { 10, 1 },
  { 11, 1 },
  { 12, 1 },
  { 1, 0 },
};

// Quaternion helper functions
Quaternion quaternion_multiply(Quaternion a, Quaternion b) {
  Quaternion result;
  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return result;
}

Quaternion quaternion_conjugate(Quaternion q) {
  Quaternion result;
  result.w = q.w;
  result.x = -q.x;
  result.y = -q.y;
  result.z = -q.z;
  return result;
}

void quaternion_normalize(Quaternion* q) {
  float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  if (norm > 0.0) {
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
  }
}

Quaternion quaternion_from_axis_angle(float x, float y, float z, float angle) {
  Quaternion result;
  float half_angle = angle * 0.5;
  float sin_half = sin(half_angle);
  result.w = cos(half_angle);
  result.x = x * sin_half;
  result.y = y * sin_half;
  result.z = z * sin_half;
  return result;
}

void update_quaternion_from_gyro(float gx, float gy, float gz, float dt) {
  // Convert gyro readings to rad/s
  gx = gx * PI / 180.0;
  gy = gy * PI / 180.0;
  gz = gz * PI / 180.0;
  
  // Create quaternion from angular velocities
  float angle = sqrt(gx*gx + gy*gy + gz*gz) * dt;
  if (angle > 0.0) {
    float axis_x = gx / sqrt(gx*gx + gy*gy + gz*gz);
    float axis_y = gy / sqrt(gx*gx + gy*gy + gz*gz);
    float axis_z = gz / sqrt(gx*gx + gy*gy + gz*gz);
    
    Quaternion delta_q = quaternion_from_axis_angle(axis_x, axis_y, axis_z, angle);
    q = quaternion_multiply(q, delta_q);
    quaternion_normalize(&q);
  }
}

void update_quaternion_from_accel(float ax, float ay, float az) {
  // Normalize accelerometer data
  float norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm == 0.0) return;
  ax /= norm;
  ay /= norm;
  az /= norm;
  
  // Calculate roll and pitch from accelerometer
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay*ay + az*az));
  
  // Create quaternion from roll and pitch (yaw is maintained from gyro)
  Quaternion q_accel;
  q_accel.w = cos(roll/2) * cos(pitch/2);
  q_accel.x = sin(roll/2) * cos(pitch/2);
  q_accel.y = cos(roll/2) * sin(pitch/2);
  q_accel.z = 0.0; // No yaw correction from accelerometer
  
  // Apply complementary filter
  q.w = FILTER_ALPHA * q.w + (1.0 - FILTER_ALPHA) * q_accel.w;
  q.x = FILTER_ALPHA * q.x + (1.0 - FILTER_ALPHA) * q_accel.x;
  q.y = FILTER_ALPHA * q.y + (1.0 - FILTER_ALPHA) * q_accel.y;
  // Keep z component from gyro integration
  
  quaternion_normalize(&q);
}

float get_handle_rotation_angle() {
  // Calculate relative quaternion from initial position
  Quaternion q_initial_conjugate = quaternion_conjugate(q_initial);
  Quaternion q_relative = quaternion_multiply(q_initial_conjugate, q);
  
  // Extract Z-axis (yaw) rotation angle
  float yaw = atan2(2.0 * (q_relative.w * q_relative.z + q_relative.x * q_relative.y),
                   1.0 - 2.0 * (q_relative.y * q_relative.y + q_relative.z * q_relative.z));
  
  return yaw;
}

void handle_func() {
  if (handle_val == 0) {
    if (pushed[RIGHT]) {
      Keyboard.release(KEY_RIGHT_ARROW);
    }
    if (pushed[LEFT]) {
      Keyboard.release(KEY_LEFT_ARROW);
    }
    handle_pushed = false;
    handle_n_pushed = 0;
    handle_n_released = 0;
    return;
  }
  int n_on = duty_ratio_arr[abs(handle_val) - 1][0];
  int n_off = duty_ratio_arr[abs(handle_val) - 1][1];
  int key = KEY_RIGHT_ARROW;
  int other_key = KEY_LEFT_ARROW;
  int key_idx = RIGHT;
  if (handle_val < 0) {
    key = KEY_LEFT_ARROW;
    other_key = KEY_RIGHT_ARROW;
    key_idx = LEFT;
  }
  if (pushed[key_idx ^ 1]) {
    Keyboard.release(other_key);
    pushed[key_idx ^ 1] = true;
  }
  if (handle_pushed) {
    if (handle_n_pushed < n_on) {
      if (handle_n_pushed == 0) {
        Keyboard.press(key);
        pushed[key_idx] = true;
      }
      ++handle_n_pushed;
    } else {
      if (n_off > 0) {
        Keyboard.release(key);
        pushed[key_idx] = false;
      }
      handle_n_pushed = 0;
      handle_n_released = 1;
      handle_pushed = false;
    }
  } else {
    if (handle_n_released < n_off) {
      if (handle_n_released == 0) {
        Keyboard.release(key);
        pushed[key_idx] = false;
      }
      ++handle_n_released;
    } else {
      Keyboard.press(key);
      pushed[key_idx] = true;
      handle_n_released = 0;
      handle_n_pushed = 1;
      handle_pushed = true;
    }
  }
}

void setup() {
  pinMode(HANDLE_PIN, INPUT);
  pinMode(SPACE_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(CALIBRATION_PIN, INPUT_PULLUP);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < N_LED; ++j) {
      pinMode(led_pins[i][j], OUTPUT);
    }
  }
  Keyboard.begin();
  // Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  // Configure accelerometer range to ±2g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // set accelerometer range to ±2g
  Wire.endTransmission(true);
  
  // Configure gyroscope range to ±2000 deg/s for fast rotation
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x18);  // set gyroscope range to ±2000 deg/s (11 on bits 4 and 3)
  Wire.endTransmission(true);
  
  // Configure low pass filter
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // set DLPF to 44Hz
  Wire.endTransmission(true);
  
  delay(500);

  // Initialize quaternion
  q.w = 1.0;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q_initial = q;

  MsTimer2::set(25, handle_func);
  MsTimer2::start();
}

int get_handle_val() {
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Convert raw data to physical units
  float ax = (float)AcX / ACCEL_SCALE;
  float ay = (float)AcY / ACCEL_SCALE;
  float az = (float)AcZ / ACCEL_SCALE;
  float gx = (float)GyX / GYRO_SCALE;
  float gy = (float)GyY / GYRO_SCALE;
  float gz = (float)GyZ / GYRO_SCALE;

  // Update quaternion using sensor fusion
  update_quaternion_from_gyro(gx, gy, gz, DT);
  update_quaternion_from_accel(ax, ay, az);

  // Get handle rotation angle
  handle_angle = get_handle_rotation_angle();

  // Convert angle to discrete handle value
  float angle_degrees = handle_angle * 180.0 / PI;
  int handle_val_p = (int)(angle_degrees / 10.0);  // 10 degrees per step
  
  // Apply deadzone
  if (abs(handle_val_p) < UNSENSE_HANDLE_VAL) {
    handle_val_p = 0;
  } else if (handle_val_p > 0) {
    handle_val_p -= UNSENSE_HANDLE_VAL;
  } else if (handle_val_p < 0) {
    handle_val_p += UNSENSE_HANDLE_VAL;
  }
  
  // Clamp to valid range
  handle_val_p = max(-N_DUTY_RATIO + 1, handle_val_p);
  handle_val_p = min(N_DUTY_RATIO - 1, handle_val_p);
  
  // Debug output (uncomment if needed)
  Serial.print("Angle: ");
  Serial.print(angle_degrees);
  Serial.print(" Handle: ");
  Serial.println(handle_val_p);
  
  return handle_val_p;
}

bool space_pressed = false;
bool brake_pressed = false;

void loop() {
  // Calibration: Set current orientation as neutral position
  if (!digitalRead(CALIBRATION_PIN)) {
    q_initial = q;  // Store current quaternion as initial orientation
    handle_angle = 0.0;
    calibration_done = true;
    
    // LED feedback for calibration
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < N_LED; ++j) {
        digitalWrite(led_pins[i][j], HIGH);
      }
    }
    delay(200);
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < N_LED; ++j) {
        digitalWrite(led_pins[i][j], LOW);
      }
    }
  }

  if (!digitalRead(SPACE_PIN)) {
    if (!space_pressed) {
      Keyboard.press(KEY_SPACE);
    }
    space_pressed = true;
    // delay(50);
  } else if (space_pressed) {
    Keyboard.release(KEY_SPACE);
    space_pressed = false;
  }

  if (!digitalRead(BRAKE_PIN)) {
    if (!brake_pressed) {
      Keyboard.press(KEY_DOWN_ARROW);
    }
    brake_pressed = true;
    // delay(50);
  } else if (brake_pressed){
    Keyboard.release(KEY_DOWN_ARROW);
    brake_pressed = false;
  }

  // int handle = analogRead(HANDLE_PIN);
  // int handle_val_p = (handle - HANDLE_NEUTRAL) / HANDLE_DIV;
  // if (abs(handle_val_p) < UNSENSE_HANDLE_VAL) {
  //   handle_val_p = 0;
  // } else if (handle_val_p > 0) {
  //   handle_val_p -= UNSENSE_HANDLE_VAL;
  // } else if (handle_val_p < 0) {
  //   handle_val_p += UNSENSE_HANDLE_VAL;
  // }
  handle_val = get_handle_val();
  if (handle_val == 0) {
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < N_LED; ++j) {
        digitalWrite(led_pins[i][j], LOW);
      }
    }
  } else {
    const int sgn[2] = {1, -1}; // right, left
    for (int i = 0; i < 2; ++i) {
      if (handle_val * sgn[i] > 0) {
        int n_led_light = min(N_LED, (handle_val * sgn[i] + 3 - 1) / 3);
        for (int j = 0; j < n_led_light; ++j) {
          digitalWrite(led_pins[i][j], HIGH);
        }
        for (int j = n_led_light; j < N_LED; ++j) {
          digitalWrite(led_pins[i][j], LOW);
        }
        for (int j = 0; j < N_LED; ++j) {
          digitalWrite(led_pins[i ^ 1][j], LOW);
        }
      }
    }
  }
  // Serial.print(handle);
  // Serial.print('\t');
  // Serial.println(handle_val_p);
  // Serial.print('\t');
  // Serial.println(handle_status);
}