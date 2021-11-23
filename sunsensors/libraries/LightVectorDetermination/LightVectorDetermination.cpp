#include "Arduino.h"
#include "SqrtFit.h"
#include "LightVectorDetermination.h"


LightVectorDetermination::LightVectorDetermination(int n_diodes, int n_readings, int pin_offset, int n_avg) {
  this->n_photodiode = n_diodes;
  this->n_readings = n_readings;
  this->photo_pin_offset = pin_offset;
  this->n_avg = n_avg;
  light_models = new SqrtFit[n_diodes];
}

void LightVectorDetermination::fit(double ** voltages) {
  /* Fits a SqrtFit model to each photodiode. The input should be a n_photodiode x n_readings matrix */
  for (int i = 0; i < n_photodiode; i++) {
    light_models[i] = SqrtFit(n_readings);
    light_models[i].fit((double *)voltages + n_readings * i);
  }
}

float LightVectorDetermination::get_global_angle() {
  double voltages[n_photodiode];
  this->read_photodiode_array(voltages);
  return this->get_global_angle(voltages);
}

float LightVectorDetermination::get_global_angle(double * voltages) {
  /* Given photodiode array readings, compute angle of incidence in range [-180,180] */
  float pred_angles[n_photodiode];  // predicted angles from each photodiode

  // Compute predicted angles for each photodiode
  for (int i = 0; i < n_photodiode; i++)
    pred_angles[i] = light_models[i].get_angle(voltages[i]);

  int f_1_idx = 0;  // index of first satellite face with incident light
  int f_2_idx = 1;  // index of second satellite face with incident light
  // Find the indices of the two most illuminated faces (i.e. find the two smallest angles)
  for (int i = 1; i < n_photodiode; i++) {
    if (pred_angles[i] < pred_angles[f_1_idx]) {  // new minimum
      f_2_idx = f_1_idx;
      f_1_idx = i;
    } else if (pred_angles[i] < pred_angles[f_2_idx])  // new 2nd minimum
      f_2_idx = i;
  }

  // Sort so that f_1_idx is the first face
  if (f_1_idx > f_2_idx) {
    int tmp = f_1_idx;
    f_1_idx = f_2_idx;
    f_2_idx = tmp;
  }

  // Handle the [270 , 360] deg range
  if (f_1_idx == 0 && f_2_idx == 3) {
    f_1_idx = 3;
    f_2_idx = 4;
  }

  // Compute angles based on their face's offset and the known direction
  float angle_1 = 90.0 * f_1_idx + pred_angles[f_1_idx];
  float angle_2 = 90.0 * f_2_idx - pred_angles[f_2_idx];
  // Average the two angles
  float result = (angle_1 + angle_2) / 2.0;
  if (result > 180)
    result = result - 360;
  return result;
}

void LightVectorDetermination::set_params(double m, double b, double max_v) {
  for (int i = 0; i < n_photodiode; i++)
    light_models[i].set_params(m, b, max_v);
}

void LightVectorDetermination::read_photodiode_array(double * voltages) {
  /* Populates a 1D voltage array with photodiode array data */
  for (int i = 0; i < n_photodiode; i++) {
    int photo_pin = i + photo_pin_offset;
    voltages[i] = _read_photodiode(photo_pin);
  }
}

void LightVectorDetermination::read_photodiode_array(double ** voltages, int col) {
  /* Populates 2D voltage array with photodiode data and prints to serial in csv form (used for calibration) */
  for (int i = 0; i < n_photodiode; i++) {
    int photo_pin = i + photo_pin_offset;

    double * temp_row = (double *)voltages + i * n_readings; // point to desired row
    temp_row[col] = _read_photodiode(photo_pin);

    if (i == n_photodiode - 1) // last photodiode - print new line
      Serial.println(temp_row[col]);
    else {
      Serial.print(temp_row[col]);
      Serial.print(",");
    }
  }
}

double LightVectorDetermination::_read_photodiode(int photo_pin){
  /* Reads a photo diode and returns the averaged signal */
  double reading_avg = 0.0;
  for (int j = 0; j < n_avg; j++)
    reading_avg += (double) analogRead(photo_pin) / n_avg;

  return reading_avg; 
}

