#include "Arduino.h"
#include "QTRSensors.h"
#include <L298NX2.h>

QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
const uint8_t SENSORS[] = {13, 12, 14, 27, 26, 25, 33, 32};
uint16_t sensorValues[SENSOR_COUNT];

#define QTR_SENSOR_COUNT 8   // Number of QTR sensors
#define QTR_EMITTER_PIN 13   // Pin connected to QTR sensor emitters
#define QTR_LINE_MID_VALUE 3500  // Example mid value from QTR sensor calibration
#define LFR_MAX_MOTOR_SPEED 125  // Maximum PWM Duty Cycle


uint16_t LFR_SensorValue[QTR_SENSOR_COUNT];   // Array to store sensor values
int16_t LFR_Position = 0;
int16_t LFR_Proportional = 0;
int16_t LFR_LastProportional = 0;
int16_t LFR_Derivative = 0;
int64_t LFR_Integral = 0;
int16_t LFR_ControlOutput = 0;


float Kp = 0.5;
float Ki = 0.001;
float Kd = 0.01;

int lastError = 0;

const uint8_t MAX_SPEED_A = 255;
const uint8_t MAX_SPEED_B = 255;
const uint8_t BASE_SPEED_A = 165;
const uint8_t BASE_SPEED_B = 165;

const uint8_t in1 = 18;
const uint8_t in2 = 19;
const uint8_t in3 = 22;
const uint8_t in4 = 23;
const uint8_t ena = 21;
const uint8_t enb = 5;
L298NX2 motors(ena, in1, in2, enb, in3, in4);

const uint8_t BUTTON = 16;
bool isOn = false;

void calibration()
{
    motors.setSpeed(0, 0);  // Stop motors
  digitalWrite(2, HIGH);
  for (uint16_t i = 0; i < 400; i++)
    qtr.calibrate();
  digitalWrite(2, LOW);
}
void LFR_Initialize() {
  qtr.init(QTR_EMITTER_PIN, QTR_SENSOR_COUNT);  // Initialize QTR sensors
  motors.init();                                // Initialize motors
  delay(2000);                                  // Optional delay for manual alignment
}

void PID_control()
{
    LFR_Position = qtrSensors.readLine(sensorValues);
  //uint16_t position = ((analogRead(SENSORS[0]) > 3500 && analogRead(SENSORS[7]) > 3500) ? qtr.readLineWhite(sensorValues) : qtr.readLineBlack(sensorValues));
  LFR_Proportional = LFR_Position - QTR_LINE_MID_VALUE;
  LFR_Derivative = LFR_Proportional - LFR_LastProportional;
  LFR_Integral += LFR_Proportional;
  LFR_LastProportional = LFR_Proportional;

  LFR_ControlOutput = LFR_Proportional / Kp + LFR_Integral / Ki + LFR_Derivative * Kd;

  if (LFR_ControlOutput > Speed) {
    LFR_ControlOutput = Speed;
  }
  if (LFR_ControlOutput < -Speed) {
    LFR_ControlOutput = -Speed;
  }

  if (LFR_ControlOutput < 0) {
    motors.setSpeed(Speed + LFR_ControlOutput, Speed);
  } else {
    motors.setSpeed(Speed, Speed - LFR_ControlOutput);
}
}

void setup()
{
    LFR_Initialize();
  analogReadResolution(8);
  Serial.begin(115200);
  Serial.println("Start!");
  qtr.setTypeAnalog();

  qtr.setSensorPins(SENSORS, SENSOR_COUNT);

  delay(250);
  pinMode(2, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  calibration();
}

void loop()
{
  if (digitalRead(BUTTON) == LOW)
    isOn = !isOn;
  if (isOn)
  {
    PID_control();
    digitalWrite(2, HIGH);
  }
  else
  {
    motors.stop();
    digitalWrite(2, LOW);
  }
  // if (isOn)
}
