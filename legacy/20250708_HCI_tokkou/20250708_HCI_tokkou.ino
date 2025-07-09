#include "Keyboard.h"
#include <MsTimer2.h>

#define HANDLE_PIN A0
#define SPACE_PIN 4
#define BRAKE_PIN 5

#ifndef KEY_SPACE
#define KEY_SPACE 0x20
#endif

#define HANDLE_DIV 50
#define UNSENSE_VAL 2

#define HANDLE_NEUTRAL 477
int handle_val = 0;
bool handle_pushed = false;
int handle_n_pushed = 0;
int handle_n_released = 0;
#define RIGHT 0
#define LEFT 1
bool pushed[2] = {false, false};

const int duty_ratio_arr[20][2] = {
  { 1, 8 },
  { 1, 6 },
  { 1, 4 },
  { 1, 3 },
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
  { 1, 0 },
  { 1, 0 },
  { 1, 0 },
  { 1, 0 },
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
  Keyboard.begin();
  // Serial.begin(9600);
  MsTimer2::set(30, handle_func);
  MsTimer2::start();
}

bool space_pressed = false;
bool brake_pressed = false;

void loop() {
  if (!digitalRead(SPACE_PIN)) {
    if (!space_pressed) {
      Keyboard.press(KEY_SPACE);
    }
    space_pressed = true;
    delay(50);
  } else if (space_pressed) {
    Keyboard.release(KEY_SPACE);
    space_pressed = false;
  }

  if (!digitalRead(BRAKE_PIN)) {
    if (!brake_pressed) {
      Keyboard.press(KEY_DOWN_ARROW);
    }
    brake_pressed = true;
    delay(50);
  } else if (brake_pressed){
    Keyboard.release(KEY_DOWN_ARROW);
    brake_pressed = false;
  }

  int handle = analogRead(HANDLE_PIN);
  int handle_val_p = (handle - HANDLE_NEUTRAL) / HANDLE_DIV;
  if (abs(handle_val_p) < UNSENSE_VAL) {
    handle_val_p = 0;
  } else if (handle_val_p > 0) {
    handle_val_p -= UNSENSE_VAL;
  } else if (handle_val_p < 0) {
    handle_val_p += UNSENSE_VAL;
  }
  handle_val = handle_val_p;
  // Serial.print(handle);
  // Serial.print('\t');
  // Serial.print(handle_val);
  // Serial.print('\t');
  // Serial.println(handle_status);
}