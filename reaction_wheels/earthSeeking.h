#ifndef earthSeek
#define earthSeek

using namespace std;
#include "Adafruit_MLX90393.h"
#include <math.h>

#define MLX90393_CS 10

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

//control pins
int enablePin = 8;
int MotorPin = 7;
int dirnPin = 6;

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

//for control with magnet
float gamma = 0.0;  // offset of normal from the y direction (depends on orientation of magnetometer in chassis)
float Bx, By, Bz;
float magnetAngle = 0;  // Angle towards earth

void setupEarthSeeking(double TargetAngle){
  //Serial.begin(115200);

  //target angle offset from normal
  Setpoint = TargetAngle;

  //set up pins
  pinMode(enablePin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(dirnPin, OUTPUT);
  digitalWrite(enablePin,1);

  if (! sensor.begin_I2C()) {         
    bt.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
  else{
    bt.println("Found a MLX90393 sensor");
  }

  sensor.setGain(MLX90393_GAIN_2_5X);

  // Set resolution, per axis
  sensor.setResolution(MLX90393_X, MLX90393_RES_19);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_19);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor.setOversampling(MLX90393_OSR_2);

  // Set digital filtering
  sensor.setFilter(MLX90393_FILTER_6);
}

void readBdirection(){
  //updates Bx, By, Bz
  if(sensor.readData(&Bx, &By, &Bz)) {
       bt.print("X: "); bt.print(Bx, 4); bt.print(" uT\t");
       bt.print("Y: "); bt.print(By, 4); bt.print(" uT\t");
       bt.print("Z: "); bt.print(Bz, 4); bt.println(" uT");
  }
  else {
        bt.println("Unable to read XYZ data from the sensor.");
  }
}

double getEarthAngle(){
  //get angle to earth from B field
  //stores in "angle"
  float xy_angle = atan(Bx / By) * 180 / 3.14159;

  if (By > 0){
    angle = xy_angle + 90;
  }else{
    angle = xy_angle - 90;
  }
  return angle;
}

void updatePID(double angle){
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

#endif