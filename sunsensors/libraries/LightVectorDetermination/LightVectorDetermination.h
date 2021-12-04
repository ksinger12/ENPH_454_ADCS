#ifndef LightVectorDetermination_h
#define LightVectorDetermination_h

#include "Arduino.h"
#include "SqrtFit.h"
#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

#define TWO_PI 6.283185307179586476925286766559

extern MPU6050 mpu;
extern SoftwareSerial bt;

class LightVectorDetermination {
public:
    SqrtFit* light_models;
    SqrtFit* fine_models;
    int n_photodiode;
    int n_readings;
    int photo_pin_offset;
    int n_avg;
    int fine_offset;
    double prev_z_angular_velocity;

    // TEST

    LightVectorDetermination(int n_photodiode, int n_readings, int pin_offset, int n_avg);
    void fit(double** voltages);
    float get_global_angle();
    float get_global_angle(double * voltages);
    //float get_fine_angle();
    void set_params(double m, double b, double max_v);
    void read_photodiode_array(double * voltages);
    void read_photodiode_array(double * voltages, int n, int offset);
    void read_photodiode_array(double ** voltages, int col);
    double _read_photodiode(int pin);
    void auto_calibrate();
    double get_z_gryo();
};

#endif
