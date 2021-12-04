#ifndef sunSeek
#define sunSeek

#include <SqrtFit.h>
#include <LightVectorDetermination.h>

int enablePin = 8;
int MotorPin = 11;
int dirnPin = 10;

double Setpoint;
double PWM_out;
double angle;

//PID control params
//gains
double kp = 0.75;
double ki = 0.002;
double kd = 10;
//input, setpoint, output
double Setpoint = 0;
double PWM_out;
double angle;
//for manual PID implementation
double cumError, rateError;
double lastError;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;

//calibrated LVD instance to read angles 
extern LightVectorDetermination LVD;

void setupSunSeeking(double TargetAngle){
  Serial.begin(9600);

  //target angle offset from normal
  Setpoint = TargetAngle;

  //set up pins
  pinMode(enablePin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(dirnPin, OUTPUT);
  digitalWrite(enablePin,1);

}

void getSunAngle(){
  //calculate desired angle change with photodiodes
  angle = LVD.get_global_angle(); 

}

void updatePID(){
  currentTime = millis();
  elapsedTime = currentTime - previousTime;

  //calc errors
  error = angle - Setpoint;
  cumError += (error * elapsedTime) / 1000;
  rateError = (error - lastError) / elapsedTime;

  //controller output
  PWM_out = kp * error + ki * cumError + kd * rateError;

  //used for next iteration of controller
  lastError = error;
  previousTime = currentTime;
}

void actuateMotor(){
  //actuate motor based on PWM_out
  if(PWM_out<0) {
    digitalWrite(dirnPin, 0); 
    PWM_out = abs(PWM_out);
    PWM_out = constrain(PWM_out,26, 229);
    analogWrite(MotorPin, PWM_out);  
  }
  else {
    digitalWrite(dirnPin, 1); 
    PWM_out = constrain(PWM_out,26, 229);
    analogWrite(MotorPin, PWM_out); 
  }

}