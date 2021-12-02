#include <SqrtFit.h>
#include <LightVectorDetermination.h>

int enablePin = 8;
int MotorPin = 7;
int dirnPin = 6;

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


void setupSunSeeking(double TargetAngle){
  Serial.begin(115200);

  //target angle offset from normal
  Setpoint = TargetAngle;

  //set up pins
  pinMode(enablePin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(dirnPin, OUTPUT);
  digitalWrite(enablePin,1);

}