#include <Arduino.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int PWMA = 22, AIN1 = 5, AIN2 = 18; // Right
const int PWMB = 23, BIN1 = 19, BIN2 = 21; // Left
const int PWM_FREQ = 5000, PWM_RES = 8;
const int PWM_CHANNEL_A = 1, PWM_CHANNEL_B = 0;
const int MAX_SPEED = 255;
const int TURN_SPEED = 180;      // normal forward or backward speed
const int SLOW_SPEED = 80;       // reduced speed for gentle turning

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car");
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  // Left
  if (leftSpeed >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    ledcWrite(PWM_CHANNEL_B, leftSpeed);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    ledcWrite(PWM_CHANNEL_B, -leftSpeed);
  }
  // Right
  if (rightSpeed >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(PWM_CHANNEL_A, rightSpeed);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    ledcWrite(PWM_CHANNEL_A, -rightSpeed);
  }
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();
    if (command == "FORWARD") {
      moveMotors(TURN_SPEED, TURN_SPEED);
    } else if (command == "BACKWARD") {
      moveMotors(-TURN_SPEED, -TURN_SPEED);
    } else if (command == "LEFT") {
      // Rotate left in place
      moveMotors(TURN_SPEED, -TURN_SPEED);
    } else if (command == "RIGHT") {
      // Rotate right in place
      moveMotors(-TURN_SPEED, TURN_SPEED);
    } else if (command == "FORWARD_LEFT") {
      // Gentle left while moving forward (slow left, fast right)
      moveMotors(TURN_SPEED, SLOW_SPEED);
    } else if (command == "FORWARD_RIGHT") {
      // Gentle right while moving forward (fast left, slow right)
      moveMotors(SLOW_SPEED, TURN_SPEED);
    } else if (command == "BACKWARD_LEFT") {_
      // Gentle left while moving backward
      moveMotors(-TURN_SPEED, -SLOW_SPEED);
    } else if (command == "BACKWARD_RIGHT") {
      // Gentle right while moving backward
      moveMotors(-SLOW_SPEED, -TURN_SPEED);
    } else if (command == "STOP") {
      moveMotors(0,0);
    }
    // Add more if needed
  }
  delay(10); // smooth and safe
}
