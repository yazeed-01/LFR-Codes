#include <QTRSensors.h>

#define Kp 0.4
#define Kd 2
#define rightMaxSpeed 114
#define leftMaxSpeed 115
#define rightBaseSpeed 85
#define leftBaseSpeed 86 


//Motor A
int PWMA = 5; //for speed control
int AIN1 = 11; //Direction
int AIN2 = 12; //Direction
//Motor B
int PWMB = 3; //for speed control
int BIN1 = 9; //Direction
int BIN2 = 8; //Direction


QTRSensors qtr;

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];
int lastError = 0;



void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);


  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH); 


  int i;
  for (int i = 0; i < 100; i++) 
  {
    if ( i  < 25 || i >= 75 ) 
    {
      turn_right();
    }
    else 
    {
      turn_left();
    }
    qtr.calibrate();
    delay(20);
  }
  //wait();
  //digitalWrite(LED_BUILTIN, LOW); 

     delay(3000); 
}

void loop()
{

  unsigned int position = qtr.readLineBlack(sensorValues);
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  

    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, rightMotorSpeed);

    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, leftMotorSpeed);
  
}
void wait() {
      digitalWrite(rightMotor1,LOW);
      digitalWrite(rightMotor2, LOW);
   

      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);
    
}
void turn_left() 
{
    digitalWrite(rightMotor1,HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite (rightMotorPWM,rightBaseSpeed); 

    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite (leftMotorPWM,leftBaseSpeed);
}
void turn_right() 
{
    digitalWrite(rightMotor1,LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite (rightMotorPWM,rightBaseSpeed);   

    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite (leftMotorPWM,leftBaseSpeed);    
}
