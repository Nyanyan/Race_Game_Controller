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
  {A3, A1, A0, 15, A2}, // right
  {6, 4, 5, 8, 9} // left
};

#ifndef KEY_SPACE
#define KEY_SPACE 0x20
#endif

// #define HANDLE_DIV 50
// #define UNSENSE_HANDLE_VAL 2
#define UNSENSE_HANDLE_VAL 2
#define GYRO_NOT_SENSE_OFFSET 80
#define ACCEL_SCALE 16384.0  // ±2g scale factor

double sum_gyz = 0;

// Variables for handle plane angle calculation
float initial_accel_x = 0, initial_accel_y = 0, initial_accel_z = 0;
bool calibrated = false;

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
  
  // Configure gyroscope range to ±2000 deg/s
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x18);  // set gyroscope range to ±2000 deg/s (11 on bits 4 and 3)
  Wire.endTransmission(true);
  
  delay(500);
  
  // Perform initial calibration
  perform_initial_calibration();

  MsTimer2::set(40, handle_func);
  MsTimer2::start();
}

void perform_initial_calibration() {
  // Read initial accelerometer values for calibration
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();
  Tmp = Wire.read()<<8|Wire.read(); // Skip temperature
  GyX = Wire.read()<<8|Wire.read();
  GyY = Wire.read()<<8|Wire.read();
  GyZ = Wire.read()<<8|Wire.read();
  
  // Convert to physical units and normalize
  float ax = (float)AcX / ACCEL_SCALE;
  float ay = (float)AcY / ACCEL_SCALE;
  float az = (float)AcZ / ACCEL_SCALE;
  
  float norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm > 0.0) {
    initial_accel_x = ax / norm;
    initial_accel_y = ay / norm;
    initial_accel_z = az / norm;
    calibrated = true;
  }
}

float calculate_handle_plane_angle() {
  if (!calibrated) return 0.0;
  
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();
  Tmp = Wire.read()<<8|Wire.read(); // Skip temperature
  GyX = Wire.read()<<8|Wire.read();
  GyY = Wire.read()<<8|Wire.read();
  GyZ = Wire.read()<<8|Wire.read();
  
  // Convert to physical units and normalize current accelerometer reading
  float ax = (float)AcX / ACCEL_SCALE;
  float ay = (float)AcY / ACCEL_SCALE;
  float az = (float)AcZ / ACCEL_SCALE;
  
  float norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm == 0.0) return 0.0;
  ax /= norm;
  ay /= norm;
  az /= norm;
  
  // Calculate initial plane normal (initial_accel × sensor_z)
  // sensor_z = (0, 0, 1)
  float initial_plane_normal_x = initial_accel_y * 1.0 - initial_accel_z * 0.0;  // = initial_accel_y
  float initial_plane_normal_y = initial_accel_z * 0.0 - initial_accel_x * 1.0;  // = -initial_accel_x
  float initial_plane_normal_z = initial_accel_x * 0.0 - initial_accel_y * 0.0;  // = 0
  
  // Normalize initial plane normal
  float initial_norm = sqrt(initial_plane_normal_x * initial_plane_normal_x + 
                           initial_plane_normal_y * initial_plane_normal_y + 
                           initial_plane_normal_z * initial_plane_normal_z);
  if (initial_norm > 0.001) {
    initial_plane_normal_x /= initial_norm;
    initial_plane_normal_y /= initial_norm;
    initial_plane_normal_z /= initial_norm;
  }
  
  // Calculate current plane normal (current_accel × sensor_z)
  float current_plane_normal_x = ay * 1.0 - az * 0.0;  // = ay
  float current_plane_normal_y = az * 0.0 - ax * 1.0;  // = -ax
  float current_plane_normal_z = ax * 0.0 - ay * 0.0;  // = 0
  
  // Normalize current plane normal
  float current_norm = sqrt(current_plane_normal_x * current_plane_normal_x + 
                           current_plane_normal_y * current_plane_normal_y + 
                           current_plane_normal_z * current_plane_normal_z);
  if (current_norm > 0.001) {
    current_plane_normal_x /= current_norm;
    current_plane_normal_y /= current_norm;
    current_plane_normal_z /= current_norm;
  }
  
  // Calculate angle between the two plane normals
  float dot_product = initial_plane_normal_x * current_plane_normal_x + 
                     initial_plane_normal_y * current_plane_normal_y + 
                     initial_plane_normal_z * current_plane_normal_z;
  
  // Clamp dot product to valid range for acos
  dot_product = constrain(dot_product, -1.0, 1.0);
  
  // Calculate cross product to determine sign
  float cross_x = initial_plane_normal_y * current_plane_normal_z - initial_plane_normal_z * current_plane_normal_y;
  float cross_y = initial_plane_normal_z * current_plane_normal_x - initial_plane_normal_x * current_plane_normal_z;
  float cross_z = initial_plane_normal_x * current_plane_normal_y - initial_plane_normal_y * current_plane_normal_x;
  
  // Use sensor Z-axis to determine rotation direction
  float sign_indicator = cross_z;  // Only Z component matters for Z-axis rotation
  
  float angle = acos(abs(dot_product));
  if (sign_indicator < 0) {
    angle = -angle;
  }
  
  return angle * 180.0 / PI;  // Convert to degrees
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

  // Calculate handle plane angle deviation
  float plane_angle = calculate_handle_plane_angle();

  // If handle is nearly vertical (within ±10 degrees), force handle to neutral
  if (abs(plane_angle) <= 10.0) {
    sum_gyz = 0;
    int handle_val_p = 0;
    
    // Debug output
    // Serial.print("Plane angle: ");
    // Serial.print(plane_angle, 1);
    // Serial.print("° Handle: ");
    // Serial.print(handle_val_p);
    // Serial.println(" (FORCED TO 0)");
    
    return handle_val_p;
  }

  double gyz_double = (GyZ - 8);
  if (abs(gyz_double) < GYRO_NOT_SENSE_OFFSET) {
    gyz_double = 0;
  } else if (gyz_double >= GYRO_NOT_SENSE_OFFSET) {
    gyz_double -= GYRO_NOT_SENSE_OFFSET;
  } else if (gyz_double <= -GYRO_NOT_SENSE_OFFSET) {
    gyz_double += GYRO_NOT_SENSE_OFFSET;
  }
  sum_gyz += gyz_double;
  int handle_val_p = sum_gyz / 25000;
  if (abs(handle_val_p) < UNSENSE_HANDLE_VAL) {
    handle_val_p = 0;
  } else if (handle_val_p > 0) {
    handle_val_p -= UNSENSE_HANDLE_VAL;
  } else if (handle_val_p < 0) {
    handle_val_p += UNSENSE_HANDLE_VAL;
  }
  handle_val_p = max(-N_DUTY_RATIO + 1, handle_val_p);
  handle_val_p = min(N_DUTY_RATIO - 1, handle_val_p);
  
  // Debug output
  // Serial.print("Plane angle: ");
  // Serial.print(plane_angle, 1);
  // Serial.print("° Handle: ");
  // Serial.println(handle_val_p);
  
  return handle_val_p;
}

bool space_pressed = false;
bool brake_pressed = false;

void loop() {
  if (!digitalRead(CALIBRATION_PIN)) {
    sum_gyz = 0;
    perform_initial_calibration();  // Re-calibrate when button pressed
    
    // Wait for button release to avoid multiple calibrations
    while (!digitalRead(CALIBRATION_PIN)) {
      delay(10);
    }
    delay(50); // Debounce delay
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