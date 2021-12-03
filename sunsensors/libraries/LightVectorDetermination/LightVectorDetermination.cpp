#include "Arduino.h"
#include "SqrtFit.h"
#include "LightVectorDetermination.h"


LightVectorDetermination::LightVectorDetermination(int n_diodes, int n_readings, int pin_offset, int n_avg, int fine_offset=4) {
  this->n_photodiode = n_diodes;
  this->n_readings = n_readings;
  this->photo_pin_offset = pin_offset;
  this->n_avg = n_avg;
  light_models = new SqrtFit[n_diodes];
  fine_models = new SqrtFit[2];
  this->fine_offset = fine_offset;
}

void LightVectorDetermination::fit(double ** voltages) {
  /* Fits a SqrtFit model to each photodiode. The input should be a n_photodiode x n_readings matrix */
  for (int i = 0; i < n_photodiode; i++) {
    light_models[i] = SqrtFit(n_readings);
    light_models[i].fit((double *)voltages + n_readings * i);
  }
  // TODO figure out calibration of fine sunsensors with the team
  for (int i = 0; i < 2; i++)
    fine_models[i] = SqrtFit(n_readings);
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
  
  if (fabs(result) < 35)
    result = get_fine_angle();

  return result;
}

float LightVectorDetermination::get_fine_angle(){
  // Computes angle of incidence from fine sunsensor. Only should be used for abs(angle) < 40
  // since the photodiodes are angled 45 deg from incidence. Assumes the photodiode analog
  // pins are sequential starting with some offset.
  // First diode should point at -45 and the second at +45
  float pred_angles[2];  // 2 fine sunsensors
  double voltages[2];

  // TODO use even time function
  read_photodiode_array(voltages, 2, fine_offset);
  for (int i = 0; i < 2; i++)
    pred_angles[i] = fine_models[i].get_angle(voltages[i]);
  
  return (pred_angles[0] - pred_angles[1]) / 2.0;  // average angles
}

void LightVectorDetermination::set_params(double m, double b, double max_v) {
  for (int i = 0; i < n_photodiode; i++)
    light_models[i].set_params(m, b, max_v);
}

void LightVectorDetermination::read_photodiode_array(double * voltages) {
  read_photodiode_array(voltages, n_photodiode, photo_pin_offset);
}

void LightVectorDetermination::read_photodiode_array(double * voltages, int n, int offset) {
  /* Populates a 1D voltage array with photodiode array data */
  for (int i = 0; i < n; i++) {
    int photo_pin = i + offset;
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

void LightVectorDetermination::auto_calibrate() {
	double cal_voltages[n_photodiode][n_readings]; 
	for(int ds=0; ds<n_readings; ds++) {
		for(int pd=0; pd<n_photodiode; pd++) {
			cal_voltages[pd][ds]= 0.0; 
		}
	}
	
    double resolution = TWO_PI / n_readings;                                        			// In radians
    double angular_velocity = get_z_gryo();                                         			// In radians/second
    unsigned long long dt = (unsigned long long)(1000*resolution / angular_velocity);    		// In seconds
	// There should be a check for division by zero/low angular velocity.
	
	Serial.println("CALIBRATING...");
	Serial.print("Angular Velocity: ");
	Serial.print(angular_velocity); 
	Serial.println(" radians/second");
	
	Serial.print("Angular Resolution: "); 
	Serial.print(resolution); 
	Serial.println(" radians"); 
	
	Serial.print("Time step: "); 
	Serial.print((long)dt); 
	Serial.println(" milliseconds");
	
    unsigned long long start_time;
    unsigned long long end_time;
    unsigned long long code_time;
    for(int i=0; i<n_readings; i++) {	

		// Get start time
		start_time = millis();

		// Loop through taking samples. 
		for(int sample = 0; sample < (int)n_avg; sample++) {
			for (int pd = 0; pd < n_photodiode; pd++) {
				//double* pd_voltage = (double*)voltages + pd*n_readings;
				int photo_pin = pd + photo_pin_offset;
				//pd_voltage[i] += (double)analogRead(photo_pin) / (double)n_avg;
				cal_voltages[pd][i] += (double)analogRead(photo_pin) / (double)n_avg;
			}
		}
		
		// Find code time 
        end_time = millis();
        code_time = (unsigned long long)((long)end_time-(long)start_time);	
		
        // Check if satellite is spinning too fast
        if(code_time>dt) {
            Serial.println("Satellite is spinning too fast to gather samples");
			return; 
        }
		
		// Find required delay
		delay((long)dt-(long)code_time);
    }
	
	Serial.println("DONE CALIBRATION"); 
  
	//this->fit((double**)cal_voltages); 
}

double LightVectorDetermination::get_z_gryo() {
    Vector angularVelocitySatellite = mpu.readNormalizeGyro();
	double z_angular_velocity = abs((double)angularVelocitySatellite.ZAxis*(TWO_PI/180.0));
    return z_angular_velocity;
}
