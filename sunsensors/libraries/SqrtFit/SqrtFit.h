#ifndef SqrtFit_h
#define SqrtFit_h

#include "Arduino.h"

class SqrtFit {
  private:

  public:
    int n_samples;
    double m;
    double b;
    double max_v;

    SqrtFit();
    SqrtFit(int);
    void fit(double voltages[]);
    void set_params(double m, double b, double max_v);
    double get_angle(double voltage);
};

#endif
