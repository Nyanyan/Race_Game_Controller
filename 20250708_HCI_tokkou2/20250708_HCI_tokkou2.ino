#include "Keyboard.h"
#include <MsTimer2.h>
#include<Wire.h>
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

// #define HANDLE_DIV 50
// #define UNSENSE_HANDLE_VAL 2
#define UNSENSE_HANDLE_VAL 2
#define GYRO_NOT_SENSE_OFFSET 80
double sum_gyz = 0;

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
  // Configure gyroscope range to ±2000 deg/s
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x18);  // set gyroscope range to ±2000 deg/s (11 on bits 4 and 3)
  Wire.endTransmission(true);
  delay(500);

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

  double gyz_double = (GyZ - 8);
  if (abs(gyz_double) < GYRO_NOT_SENSE_OFFSET) {
    gyz_double = 0;
  } else if (gyz_double >= GYRO_NOT_SENSE_OFFSET) {
    gyz_double -= GYRO_NOT_SENSE_OFFSET;
  } else if (gyz_double <= -GYRO_NOT_SENSE_OFFSET) {
    gyz_double += GYRO_NOT_SENSE_OFFSET;
  }
  sum_gyz += gyz_double;
  int handle_val_p = sum_gyz / 30000;
  if (abs(handle_val_p) < UNSENSE_HANDLE_VAL) {
    handle_val_p = 0;
  } else if (handle_val_p > 0) {
    handle_val_p -= UNSENSE_HANDLE_VAL;
  } else if (handle_val_p < 0) {
    handle_val_p += UNSENSE_HANDLE_VAL;
  }
  handle_val_p = max(-N_DUTY_RATIO + 1, handle_val_p);
  handle_val_p = min(N_DUTY_RATIO - 1, handle_val_p);
  // Serial.print(GyZ);
  // Serial.print('\t');
  // Serial.print(gyz_double);
  // Serial.print('\t');
  // Serial.print(sum_gyz);
  // Serial.print('\t');
  // Serial.println(handle_val_p);
  return handle_val_p;
}

bool space_pressed = false;
bool brake_pressed = false;

void loop() {
  if (!digitalRead(CALIBRATION_PIN)) {
    sum_gyz = 0;
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