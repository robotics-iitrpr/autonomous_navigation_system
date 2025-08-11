#include <Arduino.h>

const int pwmPin = 5;     // PWM to motor driver
const int dirPin = 6;     // Direction control
const int encA = 2;       // Encoder channel A (interrupt)
const int encB = 3;       // Encoder channel B

volatile long encoderCount = 0;

void moveMotor(int dirPin,int pwmPin,bool forward, int speed) {
  speed = constrain(speed, 0, 255);
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, speed);
  Serial.print("Motor ");
  Serial.print(forward ? "forward" : "backward");
  Serial.print(" at speed ");
  Serial.println(speed);
}

void stopMotor(int pwmPin) {
  analogWrite(pwmPin, 0);
  Serial.println("Motor stopped.");
}

void readEncoder() {
  // Basic quadrature decoding
  int A = digitalRead(encA);
  int B = digitalRead(encB);
  if (A == B)
    encoderCount++;
  else
    encoderCount--;
}


void setup() {
  // Motor control pins
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Encoder pins
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(encA), readEncoder, CHANGE);

  // Serial for control
  Serial.begin(9600);
}


void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    cmd.trim();

    if (cmd.startsWith("F")) {
      int speed = cmd.substring(1).toInt();
      moveMotor(dirPin,pwmPin,true, speed);
    } else if (cmd.startsWith("B")) {
      int speed = cmd.substring(1).toInt();
      moveMotor(dirPin,pwmPin,false, speed);
    } else if (cmd == "S") {
      stopMotor(pwmPin);
    } else if (cmd == "P") {
      Serial.print("P:");
      Serial.println(encoderCount);
    }
  }
}
