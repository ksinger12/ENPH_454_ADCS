#include <Wire.h>
#include "Adafruit_MLX90393.h"
#include <Adafruit_MCP4725.h>
#include <MPU6050.h>
#define MLX90393_CS 10

Adafruit_MLX90393 sensor = Adafruit_MLX90393();
MPU6050 mpu;
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;

float currentThroughCoils[3]; // 3 column array (3D vector -> x,y,z) -> one for each solenoid
float k = 46040; // going to need a relative magnitude value of k -> further research may be need or just example math
float kCurrent = 1; //current used to calculate k -> 1 A
float magneticField[3]; // earth's magnetic field -> 3 coumn array (3D vector -> x,y,z)
bool movement; // how are we moving -> detumbling satellite vs. detumbling motors | true = motors, false = satellite
float AREA_OF_COILS = PI*pow(0.0045,2);
int NUMBER_OF_TURNS = 61;
float resistance = 1.1; // ohms
int voltageOutputRate = 4095/5.0;

void setup() 
{
  Serial.begin(115200);

  // Initialize DACs
  dac1.begin(0x62);
  dac2.begin(0x63);

  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);

  while (!Serial) {
        delay(10);
    }

  if (! sensor.begin_I2C()) {
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

void currentCalculator(Vector gyro, float mX, float mY, float mZ) {
  float constant = k*NUMBER_OF_TURNS*AREA_OF_COILS;

  mX = mX*pow(10,-6);
  mY = mY*pow(10,-6);
  mZ = mZ*pow(10,-6);
  float angularVelocity = gyro.ZAxis*0.0174533; //rad/s

  currentThroughCoils[0] = constant*mY*angularVelocity;
  currentThroughCoils[1] = constant*mX*angularVelocity;

  
  if (currentThroughCoils[0]+currentThroughCoils[1] > kCurrent) { //use current that calculated k with
    float magnitude = sqrt(pow(currentThroughCoils[0],2) + pow(currentThroughCoils[1], 2));
    currentThroughCoils[0] = currentThroughCoils[0]/magnitude*kCurrent;
    currentThroughCoils[1] = currentThroughCoils[1]/magnitude*kCurrent;
  }

}

void loop()
{
  Vector normGyro = mpu.readNormalizeGyro();

  Serial.print(" Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normGyro.ZAxis);


  sensors_event_t event;
  sensor.getEvent(&event);

  Serial.print("\n");
  Serial.print("Bx = ");
  Serial.print(event.magnetic.x);
  Serial.print("\n");
  Serial.print("By = ");
  Serial.print(event.magnetic.y);
  Serial.print("\n");
  Serial.print("Bz = ");
  Serial.print(event.magnetic.z);
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");

  
  Serial.println("");
  currentCalculator(normGyro, event.magnetic.x, event.magnetic.y, event.magnetic.y);
  Serial.print("Ix: "); Serial.print(currentThroughCoils[0]);
  Serial.print(" \tIy: "); Serial.print(currentThroughCoils[1]);
  

  Serial.println("");
  float voltageX = resistance*currentThroughCoils[0]*voltageOutputRate;
  float voltageY = resistance*currentThroughCoils[1]*voltageOutputRate;
  Serial.print("Output: Vx: "); Serial.print(voltageX);
  Serial.print("\tOutput Vy: "); Serial.print(voltageY);
  Serial.println("");
  dac1.setVoltage(voltageX, false);
  dac2.setVoltage(voltageY, false);
    Serial.print("Vx: "); Serial.print(voltageX/voltageOutputRate);
  Serial.print("Vy: "); Serial.print(voltageY/voltageOutputRate);
  

  delay(500);
}
