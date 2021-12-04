#ifndef earthSeek
#define earthSeek

#include "Adafruit_MLX90393.h"
#include <math.h>

#define MLX90393_CS 10

extern Adafruit_MLX90393 sensor;
extern double angle;

//for control with magnet
float gamma = 0.0;  // offset of normal from the y direction (depends on orientation of magnetometer in chassis)
float Bx, By, Bz;
float magnetAngle = 0;  // Angle towards earth
float xy_angle; //stores angle to Earth from B field

void readBdirection(){
  //updates Bx, By, Bz
  if(sensor.readData(&Bx, &By, &Bz)) {
       //bt.print("X: "); bt.print(Bx, 4); bt.print(" uT\t");
       //bt.print("Y: "); bt.print(By, 4); bt.print(" uT\t");
       //bt.print("Z: "); bt.print(Bz, 4); bt.println(" uT");
  }
  else {
        bt.println("Unable to read XYZ data from the sensor.");
  }
}

void getEarthAngle(){
  //get angle to earth from B field
  //stores in "angle"
  xy_angle = atan(Bx / By) * 180 / 3.14159;

  if (By > 0){
    angle = xy_angle + 90;
  }else{
    angle = xy_angle - 90;
  }
}

#endif