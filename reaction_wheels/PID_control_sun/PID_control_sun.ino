//#include <PID_v1.h>
#include <SqrtFit.h>
#include <LightVectorDetermination.h>

//Define Variables we'll be connecting to
int enablePin = 8;
int MotorPin = 11;
int dirnPin = 10;

//control parameters
double Setpoint; //want to maintain angle of incidence of 0deg
double PWM_out; //used as output from PID controller
double angle;
double readOffset; //might use to account for measured angle error

//PID gains
double kp = 0.75;
double ki = 0.002;
double kd = 10;
double cumError, rateError;
double lastError;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;

// Define globals for photoarray
const int n_photodiode = 4;  // number of photodiodes
const int photodiode_pin_offset = 0;  // Note: we assume photodiode pin order is sequential - this is the starting pin number
const float n_average = 500.0;  // num samples to average over per reading
bool insert_header = true;

/*
 * TODO: Load calibration values (cal_voltages)
 * Reads analog 0, 1, 2, 3 with 0 pin offset
 */
const int n_cal_readings = 500;
double cal_voltages[n_photodiode][n_cal_readings];


LightVectorDetermination LVD = LightVectorDetermination(n_photodiode, 0, photodiode_pin_offset, n_average);

//Specify the links and initial tuning parameters
//PID myPID(&angle, &PWM_out, &Setpoint, kp, ki, kd, DIRECT);

void setup()
{
  //set up pins
  pinMode(enablePin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(dirnPin, OUTPUT);
  digitalWrite(enablePin,1);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
}


void loop() {
  
  //calculate desired angle change with photodiodes
  angle = LVD.get_global_angle(); 
  Serial.print("PRED ROTATION ANGLE: ");
  Serial.println(angle);

  //try running with a manual PID implementation
  //source: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  currentTime = millis();
  elapsedTime = currentTime - previousTime;

  error = angle - Setpoint;
  cumError += (error * elapsedTime) / 1000;
  
  rateError = (error - lastError) / elapsedTime;

  PWM_out = kp * error + ki * cumError + kd * rateError;

  lastError = error;
  previousTime = currentTime;


  Serial.print("PWM OUTPUT: ");
  Serial.println(PWM_out);

  //print values to plot
  Serial.print(timeFromStart);
  Serial.print(", ");
  Serial.println(error);

  if(PWM_out<0) {
    digitalWrite(dirnPin, 0); 
    PWM_out = abs(PWM_out);
    PWM_out = constrain(PWM_out,26, 229);
    //Serial.print("Writing PWM duty cycle: ");
    //Serial.println(PWM_out);
    analogWrite(MotorPin, PWM_out);  
  }
  else {
    digitalWrite(dirnPin, 1); 
    PWM_out = constrain(PWM_out,26, 229);
    //Serial.print("Writing PWM duty cycle: ");
    //Serial.println(PWM_out);
    analogWrite(MotorPin, PWM_out); 
  }
  delay(5);
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
