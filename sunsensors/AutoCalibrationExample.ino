#include <SqrtFit.h>
#include <LightVectorDetermination.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

const int n_cal_readings = 64;        // number of readings/motor steps used for calibration

// Define globals for photoarray
const int n_photodiode = 4;           // number of photodiodes
const int photodiode_pin_offset = 0;  // Note: we assume photodiode pin order is sequential - this is the starting pin number
const float n_average = 100.0;         // num samples to average over per reading
bool insert_header = true;

// Initialize LVD
LightVectorDetermination LVD = LightVectorDetermination(n_photodiode, n_cal_readings, photodiode_pin_offset, n_average);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  mpu.calibrateGyro();
}

void loop() {
  LVD.auto_calibrate();
}
