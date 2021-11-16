#include "Arduino.h"
#include "SqrtFit.h"
#include "math.h"


SqrtFit::SqrtFit() {}

SqrtFit::SqrtFit(int n_samples){
  this->n_samples = n_samples;
}

void SqrtFit::fit(double voltages[]){
  // Find index and value of max voltage
  max_v = -1;
  int argmax_v = -1;
  for (int i = 0; i < n_samples; i++){
    if (voltages[i] > max_v) {
      max_v = voltages[i];
      argmax_v = i;
    }
  }
 
  // Extract valid voltages and populate angle array
  int angle_range = 100;  // maximum angle to fit to - fit on range [0, 100]
  int n_valid = 2 * angle_range * n_samples / 360;  // number of reading in [-100, 100] deg range
  float angles[n_valid];
  double valid_voltages[n_valid];  // subset of voltages in valid range
  
  int min_valid_v_idx = argmax_v - floor(n_valid / 2);
  int max_valid_v_idx = argmax_v + ceil((float(n_valid) / 2));
  
  int j = 0;
  for (int i = min_valid_v_idx; i < max_valid_v_idx; i++){
    int idx = i % n_samples;
    valid_voltages[j] = sqrt(max_v - voltages[idx]);
    angles[j] = fabs(-1 * angle_range + j * 360 / n_samples);
    j++;
  }
  
  /* ------- Linear regression -------
    x := voltage
    y := angle
  */
  // initialize variables
  double x_bar = 0;
  double y_bar = 0;
  double xy_bar = 0;
  double xsq_bar = 0;
  
  // calculations required for linear regression
  for (int i = 0; i < n_valid; i++){
    x_bar += valid_voltages[i] / n_valid;
    y_bar += angles[i] / n_valid;
    xy_bar += valid_voltages[i] * angles[i] / n_valid;
    xsq_bar += valid_voltages[i] * valid_voltages[i] / n_valid;
  }
  m = (xy_bar - x_bar * y_bar) / (xsq_bar - x_bar * x_bar);
  b = y_bar - m * x_bar;
  
  Serial.print("theta = ");
  Serial.print(m);
  Serial.print(" * V + ");
  Serial.println(b);
}

void SqrtFit::set_params(double m, double b, double max_v){
  this->m = m;
  this->b = b;
  this->max_v = max_v;
}

double SqrtFit::get_angle(double voltage){
  if votlage > max_v
    return 0.0;
  return fabs(m * sqrt(fabs(max_v - voltage)) + b);
}

