#include <PID_v1.h>
#include <LightVectorDetermination.h>

//Define Variables we'll be connecting to
int enablePin = 8;
int MotorPin = 7;
int dirnPin = 6;

//control parameters
double Setpoint = 0; //want to maintain angle of incidence of 0deg
float PWM_out; //used as output from PID controller
float angle;

//PID gains
double kp = 2;
double ki = 0;
double kd = 0;

// Define globals for photoarray
const int n_photodiode = 4;  // number of photodiodes
const int photodiode_pin_offset = 0;  // Note: we assume photodiode pin order is sequential - this is the starting pin number
const float n_average = 500.0;  // num samples to average over per reading
bool insert_header = true;

/*
 * TODO: Load calibration values (cal_voltages)
 * Reads analog 0, 1, 2, 3 with 0 pin offset
 */


LightVectorDetermination LVD = LightVectorDetermination(n_photodiode, 0, photodiode_pin_offset, n_average);

//fit sqrtfit model to each photodiode using calibration values
LVD.fit((double **) cal_voltages);

//Specify the links and initial tuning parameters
PID myPID(&angle, &PWM_out, &Setpoint, kp, ki, kd, DIRECT);

void setup()
{
  //set up pins
  pinMode(enablePin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(dirnPin, OUTPUT);
  digitalWrite(enablePin,1);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
}


void loop() {
  
  //calculate desired angle change with photodiodes
  angle = LVD.get_global_angle(); 
  Serial.print("PRED ROTATION ANGLE: ");
  Serial.println(angle);

  //compute PID output, actuate
  myPID.Compute();
  PWM_out = constrain(PWM_out, 26, 229);

  Serial.print("PWM OUTPUT: ");
  Serial.println(PWM_out);

  //sign of PWM_out indicates directionality
  if(PWM_out<0) {
    digitalWrite(dirnPin, 0); 
    analogWrite(MotorPin, abs(PWM_out));  
  }
  else {
    digitalWrite(dirnPin, 1); 
    analogWrite(MotorPin, abs(PWM_out)); 
  }

  //delay(100);
}
/*
 * Here, we are feeding it a series of inputs which are used to tune the controller
 * The output represents the 'output' pwm to adjust the input 
 * The 2, 5, 1 values are the kp, ki, kd variables 
 * The setpoint is the value we want to maintain 
 * Direct is for going towards the correct input. That should be DIRECT 
 * 10-90 % PWM for reaction wheel 
 * Direction: digital pin for direction
 * The pin for PWM
 * The pin for Directionality
 * Angles: -180 to +180 
 * 
 */
