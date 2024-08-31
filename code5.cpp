#include "Arduino.h"
#include "QTRSensors.h"
#include <L298NX2.h>

QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
const uint8_t SENSORS[] = {13, 12, 14, 27, 26, 25, 33, 32};
uint16_t sensorValues[SENSOR_COUNT];

float Kp = 0.5;
float Ki = 0.001;
float Kd = 0.6;

int lastError = 0;
int integral = 0;

const uint8_t MAX_SPEED_A = 255;
const uint8_t MAX_SPEED_B = 255;
const uint8_t BASE_SPEED_A = 240;
const uint8_t BASE_SPEED_B = 240;

const uint8_t in1 = 18;
const uint8_t in2 = 19;
const uint8_t in3 = 22;
const uint8_t in4 = 23;
const uint8_t ena = 21;
const uint8_t enb = 5;
L298NX2 motors(ena, in1, in2, enb, in3, in4);

const uint8_t BUTTON = 16;
bool isOn = false;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 50;

void calibration() {
  digitalWrite(2, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(2, LOW);
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  if (position > 3800) {
    motors.setSpeedA(MAX_SPEED_A); // Max speed for motor A, stop motor B
    motors.forwardA();
    motors.backwardB();
    return;
  }

  if (position < 2900) {
    motors.setSpeedB(MAX_SPEED_B); // Stop motor A, max speed for motor B
    motors.forwardB();
    motors.backwardA();
    return;
  }

  int error = position - 3500;
  integral += error;
  int derivative = error - lastError;
  int motorspeed = error * Kp + integral * Ki + derivative * Kd;
  lastError = error;

  int motor_speed_a = BASE_SPEED_A + motorspeed;
  int motor_speed_b = BASE_SPEED_B - motorspeed;

  motor_speed_a = constrain(motor_speed_a, 0, MAX_SPEED_A);
  motor_speed_b = constrain(motor_speed_b, 0, MAX_SPEED_B);

  motors.setSpeedA(motor_speed_a);
  motors.setSpeedB(motor_speed_b);

  motors.forward();
}

void setup() {
  analogReadResolution(8);
  Serial.begin(115200);
  Serial.println("Start!");

  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSORS, SENSOR_COUNT);

  pinMode(2, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  
  calibration();
}

void loop() {
  if (digitalRead(BUTTON) == LOW) {
    unsigned long currentTime = millis();
    if (currentTime - lastButtonPress > debounceDelay) {
      isOn = !isOn;
      lastButtonPress = currentTime;
    }
  }

  if (isOn) {
    PID_control();
    digitalWrite(2, HIGH);
  } else {
    motors.stop();
    digitalWrite(2, LOW);
  }
}
