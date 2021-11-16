#include <PID_v1.h>

//Define Variables we'll be connecting to
float readPin = A0;
int enablePin = 8;
int MotorPin = 7;
int dirnPin = 6;

double Setpoint = 1000;
double Input, Output;

double kp = 2;
double ki = 0;
double kd = 0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

void setup()
{
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin,1);

  Serial.begin(9600);
  
  //initialize the variables we're linked to
  Input = analogRead(readPin);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}


void loop() {
  
  //reading, averaging 500 values
  Input = 0;
  for(int i=0; i<500; i++){
    Input += double(analogRead(readPin)) / 500.0;
  }

  myPID.Compute();
  Output = constrain(Output, 26, 229);
  
  Serial.print(Input);
  Serial.print(" ");
  Serial.print(Output);
  Serial.println();
  
  if(Output<0) {
    analogWrite(MotorPin, abs(Output));  
    digitalWrite(dirnPin, 0); // Direction
  }
  else {
    analogWrite(MotorPin, abs(Output)); 
    digitalWrite(dirnPin, 1); 
  }

  //delay(100);
}
/*
 * Here, we are feeding it a series of inputs which are used to tune the controller
 * The output represents the 'output' pwm to adjust the input 
 * The 2, 5, 1 values are the kp, ki, kd variables 
 * The setpoint is the value we want to maintain 
 * Direct is for going towards the correct input. That should be DIRECT 
 * 10-90 % PWM for reaction wheel 
 * Direction: digital pin for direction
 * The pin for PWM
 * The pin for Directionality
 * Angles: -180 to +180 
 * 
 */
