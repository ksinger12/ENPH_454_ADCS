#include <PID_v1.h>
#include "Adafruit_MLX90393.h"
#include <math.h>

//sensor setup
Adafruit_MLX90393 sensor = Adafruit_MLX90393();
#define MLX90393_CS 10

//control pins
int enablePin = 8;
int MotorPin = 7;
int dirnPin = 6;

//control parameters
double Setpoint = 0; //want to maintain angle of incidence of 0deg
double PWM_out; //used as output from PID controller
double angle;

//PID gains
double kp = 2;
double ki = 0;
double kd = 0;

//Specify the links and initial tuning parameters
PID myPID(&angle, &PWM_out, &Setpoint, kp, ki, kd, DIRECT);

void setup() {
  //set up pins
  pinMode(enablePin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(dirnPin, OUTPUT);
  digitalWrite(enablePin,1);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(115200);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
      delay(10);
  }

  //set up magntometer for I2C
  if (! sensor.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! sensor.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
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


  /* ----- Determine Angle -----
   *  Assume the two components we need to analyze are x and y
   *  Also assume that the normal is offset from y by some angle gamma
   */
   float gamma = 0.0;  // offset of normal from the y direction (depends on orientation of magnetometer in chassis)
   float Bx, By, Bz;
   float magnetAngle;  // Angle towards earth

  // get X Y and Z data at once
  if (sensor.readData(&Bx, &By, &Bz)) {
      Serial.print("X: "); Serial.print(Bx, 4); Serial.println(" uT");
      Serial.print("Y: "); Serial.print(By, 4); Serial.println(" uT");
      Serial.print("Z: "); Serial.print(Bz, 4); Serial.println(" uT");
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }

  float xy_angle = atan(Bx / By) * 180 / 3.14159;  // Find angle of B w.r.t the y direction
  Serial.println(xy_angle);
  if (By > 0){
    magnetAngle = xy_angle + 90;
  }else{
    magnetAngle = xy_angle - 90;
  }
  Serial.print("Angle of earth is: ");
  Serial.println(magnetAngle);

  angle = magnetAngle;

  //modify desired angle change using magnetAngle (should be pointing to the 

  //compute PID output, actuate
  myPID.Compute();
  PWM_out = constrain(PWM_out, 26, 229);

  Serial.print("PWM OUTPUT: ");
  Serial.println(PWM_out);

  //sign of PWM_out indicates directionality
  if(PWM_out<0) {
    digitalWrite(dirnPin, 0); 
    analogWrite(MotorPin, abs(PWM_out));  
  }
  else {
    digitalWrite(dirnPin, 1); 
    analogWrite(MotorPin, abs(PWM_out)); 
  }

}
