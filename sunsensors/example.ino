// Include the Arduino Stepper.h library:
#include <math.h>
#include <Stepper.h>
#include <SqrtFit.h>
#include <LightVectorDetermination.h>

// Define globals for stepper
const int steps_per_revolution = 2048;
const int steps_per_sweep = steps_per_revolution;  // full revolution
const int n_cal_readings = steps_per_sweep / 32;  // number of readings/motor steps used for calibration

// Create stepper object and set pins
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper my_stepper = Stepper(steps_per_revolution, 22, 26, 24, 28);


// Define globals for photoarray
const int n_photodiode = 4;  // number of photodiodes
const int photodiode_pin_offset = 0;  // Note: we assume photodiode pin order is sequential - this is the starting pin number
const float n_average = 500.0;  // num samples to average over per reading
bool insert_header = true;

LightVectorDetermination LVD = LightVectorDetermination(n_photodiode, n_cal_readings, photodiode_pin_offset, n_average);


void setup() {
  // Set the speed to 5 rpm:
  my_stepper.setSpeed(5);

  // Begin Serial communication at a baud rate of 9600:
  Serial.begin(9600);
}

void print_header() {
  /* Print column names to serial for csv*/
  Serial.print("angle,");

  // Iterate over photodiodes in array
  for (int i = 0; i < n_photodiode; i++) {
    if (i == n_photodiode - 1) {
      // print insert new line for last column
      Serial.print("Photodiode_");
      Serial.println(i);
    } else {
      Serial.print("Photodiode_");
      Serial.print(i);
      Serial.print(",");
    }
  }
}

void loop() {
  double cal_voltages[n_photodiode][n_cal_readings];  // calibration votlages

  if (insert_header) {
    print_header();
    insert_header = false;

    /*
     * --- CALIBRATION ---
     */
    // Rotate photo array half of full sweep in negative direction
    my_stepper.step(-steps_per_sweep / 2);

    // Voltage sweep
    for (int i = 0; i < n_cal_readings; i++) {
      delay(1000);
      // print current angle
      Serial.print(-180.0 + (float)i * 360 / n_cal_readings);
      Serial.print(",");

      // Populate calibration matrix. Note that you need to cast 2D matrices as a double pointer
      LVD.read_photodiode_array((double **)cal_voltages, i);
      my_stepper.step(steps_per_sweep / n_cal_readings);
    }
    my_stepper.step(-steps_per_sweep / 2); // rotate to initial point of -90deg
    delay(1000);

    // Fit a model to each photodiode's voltage curve
    LVD.fit((double **) cal_voltages);
  }

  /*
   * --- TESTING ---
   */
  float angle;
  double test_voltages[n_photodiode];
  
  for (int n = 0; n < 1000; n++) {
    angle = LVD.get_global_angle();
    Serial.print("PRED GLOBAL ANGLE: ");
    Serial.println(angle);

    my_stepper.step(floor(-1 * angle * steps_per_revolution / 360));
  }
  //  delay(2000);



}
