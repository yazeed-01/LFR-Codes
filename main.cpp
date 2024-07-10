#include <Arduino.h>
#include <QTRSensors.h>
#include <L298N.h>
#include <TaskScheduler.h>
#include <QuickPID.h>

#define HISTORY_SIZE 10  // Cumulative Weighted average to calculate last line detection
#define NO_LINE_COUNT 20 // How many times motor will go forward if no line is detected
// #define AGGRESSIVE_PID 1
// Motor Pin definitions
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
#define ENA 21
#define ENB 5


const uint8_t BUTTON = 16;
bool isOn = false;
const int offsetA = 1;
const int offsetB = 1;

L298N motorDriver(IN1, IN2, IN3, IN4, ENA, ENB);

int leftMinSpeed = 0;
int rightMinSpeed = 0;
int leftMaxSpeed = 255;
int rightMaxSpeed = 255;

// Sensor definitions
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
float sensorWeights[SensorCount] = {12.5, 2.5, 0.5, 0.1, 0.1, 0.5, 2.5, 12.5};

u_int16_t readingHistory[SensorCount][HISTORY_SIZE];

QTRSensors qtr;

Scheduler runner;

int32_t lineError = 0;
u_int16_t lineThreshold = 800; // Sensor value Threshold for line detection
u_int8_t noLineCount = 0;

// PID variables
float Kp = 1;
float Ki = 0.05;
float Kd = 0.25;

#ifdef AGGRESSIVE_PID
// Aggressive PID Variables
float AggresiveKp = 4;
float AggresiveKi = 0.2;
float AggresiveKd = 1;
#endif

float setPointLeft = 0;
float inputLeft = 0;
float outputLeft = 0;

float setPointRight = 0;
float inputRight = 0;
float outputRight = 0;

float prevError = 0;
int8_t direction = 0;
uint8_t pidType = 0;

QuickPID leftPid(&inputLeft, &outputLeft, &setPointLeft, Kp, Ki, Kd, QuickPID::Action::direct);
QuickPID rightPid(&inputRight, &outputRight, &setPointRight, Kp, Ki, Kd, QuickPID::Action::direct);

int prevLeftMotorSpeed = 0;
int prevRightMotorSpeed = 0;

void readAndCalculateError();
void calculateMotorSpeed();
void setMotorSpeed();
void recordHistory();
Task readAndCalculateErrorTask(10, TASK_FOREVER, &readAndCalculateError, &runner, true);
Task calculateMotorSpeedTask(10, TASK_FOREVER, &calculateMotorSpeed, &runner, true);
Task setMotorSpeedTask(200, TASK_FOREVER, &setMotorSpeed, &runner, true);
Task recordHistoryTask(30, TASK_FOREVER, &recordHistory, &runner, true);
Task printTask(20, TASK_FOREVER, &print, &runner, true);

u_int8_t findWhereLastLineWentFromHistory()
{
  int32_t historyLineError = 0;
  // Iterate over the last 10 history arrays
  for (int l = HISTORY_SIZE - 1; l >= 0; l--)
  {
    for (int i = 0; i < SensorCount; i++)
    {
      historyLineError += (i - 3.5) * sensorValues[i] * sensorWeights[i] * (l / 10);
    }
  }

  // If no line detection is found in the last 10 history arrays, return -1
  return historyLineError > 50? 1 : historyLineError < -50? -1
                                                            : 0;
}

/*
  Read And Calculate Error
  ----
  Get readings from sensor array and calculate error.
*/
void readAndCalculateError()
{
  // Read sensor values
  qtr.readCalibrated(sensorValues);

  // Check like exists
  bool lineExists = false;
  lineError = 0;
  // Calculate line error and direction
  for (int i = 0; i < SensorCount; i++)
  {
    lineError += (i - 3.5) * sensorValues[i] * sensorWeights[i];
    if (sensorValues[i] > lineThreshold)
    {
      lineExists = true;
    }
  }

  if (lineExists == false)
  {
    noLineCount++;
    if (noLineCount > NO_LINE_COUNT)
      lineError = findWhereLast
        }
  else
  {
    noLineCount = 0;
  }
}


void recordHistory()
{
  u_int8_t count = 0;
  for (int i = 0; i < SensorCount; i++)
    if (sensorValues[i] > lineThreshold)
      count++;

  if (count == 0)
    return;

  for (int i = 0; i < SensorCount; i++)
  {
    for (int j = HISTORY_SIZE - 1; j > 0; j--)
    {
      readingHistory[i][j] = readingHistory[i][j - 1];
    }
    readingHistory[i][0] = map(sensorValues[i], 0, 1000, 0, 255);
  }
}


void recordHistory()
{
  u_int8_t count = 0;
  for (int i = 0; i < SensorCount; i++)
    if (sensorValues[i] > lineThreshold)
      count++;

  if (count == 0)
    return;

  for (int i = 0; i < SensorCount; i++)
  {
    for (int j = HISTORY_SIZE - 1; j > 0; j--)
    {
      readingHistory[i][j] = readingHistory[i][j - 1];
    }
    readingHistory[i][0] = map(sensorValues[i], 0, 1000, 0, 255);
  }
}

void setMotorSpeed()
{
  // Calculate speed for both motors.
  // If line error is negative then right motor should have higher speed than left motor and vice versa.

  // Calculate motor speed
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;

  // TODO: find diffrence between previous error and calculate diffrence.
  // TODO: If diffrence is at peak, then set motor speed for sharp turn and decrease acceleration.

  leftMotorSpeed = outputLeft;
  rightMotorSpeed = outputRight;

  prevLeftMotorSpeed = leftMotorSpeed;
  prevRightMotorSpeed = rightMotorSpeed;
  // Drive motors with calculated motor speed
  motorDriver.setSpeed(0, 255 - leftMotorSpeed);
  motorDriver.setSpeed(1, 255 - rightMotorSpeed);
}

/*
 Calculate motor speed and drive motors
*/
void calculateMotorSpeed()
{
  // Calculate motor speed from line error
  inputRight = map(lineError, 0, 50000, 0, 255);
  inputLeft = map(lineError, 0, -50000, 0, 255);

#ifdef AGGRESSIVE_PID
  (inputRight > 200) ? rightPid.SetTunings(AggresiveKp, AggresiveKi, AggresiveKd) : rightPid.SetTunings(Kp, Ki, Kd);
  (inputLeft > 200) ? leftPid.SetTunings(AggresiveKp, AggresiveKi, AggresiveKd) : leftPid.SetTunings(Kp, Ki, Kd);
#endif
  rightPid.Compute();
  leftPid.Compute();
}

void setup()
{
  Serial.begin(115200);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){13, 12, 14, 27, 26, 25, 33, 32}, SensorCount); 
  delay(250);
  pinMode(2, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  for (uint8_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    digitalWrite(2, LOW);
  }
    delay(10);
  leftPid.SetMode(QuickPID::Control::automatic);
  rightPid.SetMode(QuickPID::Control::automatic);
}

void loop()
{
    if (digitalRead(BUTTON) == LOW)
    isOn = !isOn;
  if (isOn)
  {
    runner.execute();
  }
  else
  {
    motors.stop();
    digitalWrite(2, LOW);
  }
}



