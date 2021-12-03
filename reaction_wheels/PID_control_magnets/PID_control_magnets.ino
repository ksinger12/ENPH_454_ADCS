//#include <PID_v1.h>
#include "Adafruit_MLX90393.h"
#include <math.h>
#include <SoftwareSerial.h>

//set up for bluetooth shield
SoftwareSerial bt(2,3); // RX, TX

//sensor setup
Adafruit_MLX90393 sensor = Adafruit_MLX90393();
#define MLX90393_CS 10

//control pins
int enablePin = 8;
int MotorPin = 11;
int dirnPin = 10;

//control parameters
double Setpoint = 0; //want to maintain angle of incidence of 0deg
double PWM_out; //used as output from PID controller
double angle;

//PID gains
double kp = 0.75;
double ki = 0.002;
double ki = 0.005;
double kd = 10;
double cumError, rateError;
double lastError;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double timeFromStart;

//for control with magnet
float gamma = 0.0;  // offset of normal from the y direction (depends on orientation of magnetometer in chassis)
float Bx, By, Bz;
float magnetAngle = 0;  // Angle towards earth

  

void setup() {
  bt.begin(9600);
  Serial.begin(9600);
  //set up pins
  pinMode(enablePin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(dirnPin, OUTPUT);
  digitalWrite(enablePin,1);

  
  
  //set up magntometer for I2C
  if (! sensor.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
  //sensor.begin_I2C();
  Serial.println("Found a MLX90393 sensor");
  

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

void loop() {
  Serial.begin(9600);

  /* ----- Determine Angle -----
   *  Assume the two components we need to analyze are x and y
   *  Also assume that the normal is offset from y by some angle gamma
   */
   

  // get X Y and Z data at once
  if (sensor.readData(&Bx, &By, &Bz)) {
      //Serial.print("X: "); Serial.print(Bx, 4); Serial.println(" uT");
      //Serial.print("Y: "); Serial.print(By, 4); Serial.println(" uT");
      //Serial.print("Z: "); Serial.print(Bz, 4); Serial.println(" uT");
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }
 
  
  float xy_angle = atan(Bx / By) * 180 / 3.14159;  // Find angle of B w.r.t the y direction
  //Serial.println(xy_angle);
  if (By > 0){
    magnetAngle = xy_angle + 90;
  }else{
    magnetAngle = xy_angle - 90;
  }
  //Serial.print("Angle of earth is: ");
  //Serial.println(magnetAngle);

  //actuating to this angle should make it try to orient a face towards the magnet
  angle = magnetAngle; 
  
  //try running with a manual PID implementation
  //source: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  currentTime = millis();
  elapsedTime = currentTime - previousTime;

  error = angle - Setpoint;
  cumError += (error * elapsedTime) / 1000;
  //Serial.println(cumError);
  
  rateError = (error - lastError) / elapsedTime;

  PWM_out = kp * error + ki * cumError + kd * rateError;

  lastError = error;
  previousTime = currentTime;
  timeFromStart += elapsedTime;

  //print values to plot
  bt.print(timeFromStart);
  bt.print(", ");
  bt.println(error);
  //Serial.println(timeFromStart);

  //sign of PWM_out indicates directionality
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
