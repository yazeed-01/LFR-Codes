#include "Arduino.h"
#include "QTRSensors.h"
#include <L298NX2.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
const uint8_t SENSORS[] = {13, 12, 14, 27, 26, 25, 33, 32};
uint16_t sensorValues[SENSOR_COUNT];

float Kp = 0.33;
float Ki = 0.001;
float Kd = 0.6;

int lastError = 0;

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

void calibration()
{
  digitalWrite(2, HIGH);
  for (uint16_t i = 0; i < 400; i++)
    qtr.calibrate();
  digitalWrite(2, LOW);
}

void PID_control()
{
  uint16_t position = qtr.readLineBlack(sensorValues);

  if (position > 3800)
  {
    motors.setSpeed(BASE_SPEED_B);
    motors.forwardB();
    motors.backwardA();
    return;
  }

  if (position < 2900)
  {
    motors.setSpeed(BASE_SPEED_A);
    motors.forwardA();
    motors.backwardB();
    return;
  }

  int error = position - 3500;
  int P = error;
  int I = I + error;
  int D = error - lastError;
  int motorspeed = P * Kp + I * Ki + D * Kd;
  lastError = error;

  int motor_speed_a = BASE_SPEED_A + motorspeed;
  int motor_speed_b = BASE_SPEED_B - motorspeed;

  if (motor_speed_a > MAX_SPEED_A)
    motor_speed_a = MAX_SPEED_A;

  if (motor_speed_b > MAX_SPEED_B)
    motor_speed_b = MAX_SPEED_B;

  if (motor_speed_a < 0)
    motor_speed_a = 0;

  if (motor_speed_b < 0)
    motor_speed_b = 0;

  motors.forward();
}

void setup()
{
  analogReadResolution(8);
  Serial.begin(115200);
  SerialBT.begin("ESP32Test"); // Bluetooth device name
  Serial.println("Bluetooth Started! Ready to pair...");
  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSORS, SENSOR_COUNT);

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

  if (SerialBT.available())
  {
    String command = SerialBT.readStringUntil('\n');
    if (command == "START")
    {
      isOn = true;
    }
    else if (command == "STOP")
    {
      isOn = false;
    }
    else if (command.startsWith("KP"))
    {
      Kp = command.substring(2).toFloat();
    }
    else if (command.startsWith("KI"))
    {
      Ki = command.substring(2).toFloat();
    }
    else if (command.startsWith("KD"))
    {
      Kd = command.substring(2).toFloat();
    }
  }
}
