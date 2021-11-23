#ifndef LightVectorDetermination_h
#define LightVectorDetermination_h

#include "Arduino.h"
#include "SqrtFit.h"
#include <MPU6050.h>

#define TWO_PI 6.283185307179586476925286766559

class LightVectorDetermination {
  public:
    SqrtFit* light_models;
    int n_photodiode;
    int n_readings;
    int photo_pin_offset;
    int n_avg;

    LightVectorDetermination(int n_photodiode, int n_readings, int pin_offset, int n_avg);
    void fit(double** voltages);
    float get_global_angle();
    float get_global_angle(double * voltages);
    void set_params(double m, double b, double max_v);
    void read_photodiode_array(double * voltages);
    void read_photodiode_array(double ** voltages, int col);
    double _read_photodiode(int pin);
    void read_photodiode_array_even_time(double * voltages);
    void auto_calibrate(double ** voltages);
    double get_z_gryo();
};

#endif
