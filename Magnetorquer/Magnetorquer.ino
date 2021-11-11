#include <Wire.h>
#include <MPU6050.h>
#include "Adafruit_MLX90393.h"

float currentThroughCoils[3]; // 3 column array (3D vector -> x,y,z) -> one for each solenoid
Vector angularVelocityMotor;
Vector angularVelocitySatellite;
float k = 46040; // going to need a relative magnitude value of k -> further research may be need or just example math
float kCurrent = 1; //current used to calculate k -> 1 A
float magneticField[3]; // earth's magnetic field -> 3 coumn array (3D vector -> x,y,z)
bool movement; // how are we moving -> detumbling satellite vs. detumbling motors | true = motors, false = satellite
float AREA_OF_COILS = PI*pow(0.0031,2);
int NUMBER_OF_TURNS = 61;


Adafruit_MLX90393 fieldSensor = Adafruit_MLX90393();
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  movement = false; //detumbling motors
  Serial.println("Setup");

  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();
  Serial.println("Calibrate");  
  // Set threshold sensivty. Default 3.
  mpu.setThreshold(3);
  Serial.println("Threshold");
  
  // Set resolution, per axis
  fieldSensor.setResolution(MLX90393_X, MLX90393_RES_19);
  fieldSensor.setResolution(MLX90393_Y, MLX90393_RES_19);
  fieldSensor.setResolution(MLX90393_Z, MLX90393_RES_16);
  Serial.println("Field resolution");
  
  // Set oversampling
  fieldSensor.setOversampling(MLX90393_OSR_2);
  Serial.println("Field oversampling");

  // Set digital filtering
  fieldSensor.setFilter(MLX90393_FILTER_6);
  Serial.println("Field Filter");
}

void loop() { 
  Serial.println("Loop");
  //angularVelocityMotor = ; // input value read from hall sensor on reaction wheel
  angularVelocitySatellite = mpu.readNormalizeGyro(); //input value from external hall sensor
  
  float x, y, z;

  // get X Y and Z data at once
  if (fieldSensor.readData(&x, &y, &z)) {
      magneticField[0] = x; //input from magnetometer
      magneticField[1] = y;
      magneticField[2] = z;
      Serial.println("Magnetic Field.");
  } else {
      Serial.println("Unable to read Magnetic Field.");
  }

  if (movement) {
    currentCalculator(magneticField, angularVelocityMotor);
  } else {
    currentCalculator(magneticField, angularVelocitySatellite);
  }

  Serial.print("W_z = ");
  Serial.print(angularVelocitySatellite.ZAxis);
  Serial.print("Current_x = ");
  Serial.print(currentThroughCoils[0]);
  Serial.print("Current_y = ");
  Serial.print(currentThroughCoils[1]);
  Serial.print("B_x = ");
  Serial.print(magneticField[0]);
  Serial.print("B_y = ");
  Serial.print(magneticField[1]);

}

void currentCalculator(float magneticField[], Vector angularVelocity) {
  float constant = k*NUMBER_OF_TURNS*AREA_OF_COILS;

  currentThroughCoils[0] = constant*magneticField[1]*angularVelocity.ZAxis;
  currentThroughCoils[1] = constant*magneticField[0]*angularVelocity.ZAxis;

  
  if (currentThroughCoils[0]+currentThroughCoils[1] > kCurrent) { //use current that calculated k with
    float magnitude = sqrt(pow(currentThroughCoils[0],2) + pow(currentThroughCoils[1], 2));
    currentThroughCoils[0] = currentThroughCoils[0]/magnitude*kCurrent;
    currentThroughCoils[1] = currentThroughCoils[1]/magnitude*kCurrent;
  }

}


  // B is in Tesla -> 40000 nT = 4e4e-9 = 4e-5
  // angular velocity is rad/s -> say 0.1 rad/s -> 1e-1
  // current = k*cross(w,B)/(n*A) -> in amps ~= 40 mA = 4e-2
  // therefore say k = 67200

  //saturation check 
  //if the sum of the absolute value of the current is greate than 0.04
  // normalize it: current = current/normal(current)*0.04

  //now we have current -> which needs to be calculated everytime
  // magnetic dipole is then: muB = current*n*A 

  //that is how we detumble
  
  
