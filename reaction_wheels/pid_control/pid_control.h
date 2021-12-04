/*
Includes PID code and code to actuate motors 
*/


#ifndef PIDcontrol
#define PIDcontrol

//control pins
extern const int enablePin;
extern const int MotorPin;
extern const int dirnPin;

//PID control params
//gains
extern const double kp;
extern const double ki;
extern const double kd;
//input, setpoint, output
extern double Setpoint;
extern double PWM_out;
extern double angle;
//for manual PID implementation
extern double rateError;
extern double cumError, lastError;
extern unsigned long currentTime, previousTime, elapsedTime;
extern double error;


void setupPID(double target_angle){
    cumError = 0;
    elapsedTime = 0;

    Setpoint = target_angle;

    //set up pins
    pinMode(enablePin, OUTPUT);
    pinMode(MotorPin, OUTPUT);
    pinMode(dirnPin, OUTPUT);
    digitalWrite(enablePin,1);
}

void updatePID(){
  currentTime = millis();
  elapsedTime = currentTime - previousTime;

  //calc errors
  error = angle - Setpoint;
  cumError += (error * elapsedTime) / 1000;
  rateError = (error - lastError) / elapsedTime;

  //controller output
  PWM_out = kp * error + ki * cumError + kd * rateError;

  //used for next iteration of controller
  lastError = error;
  previousTime = currentTime;
}

void actuateMotor(){
  //actuate motor based on PWM_out
  if(PWM_out<0) {
    digitalWrite(dirnPin, 0); 
    PWM_out = abs(PWM_out);
    PWM_out = constrain(PWM_out,26, 229);
    analogWrite(MotorPin, PWM_out);  
  }
  else {
    digitalWrite(dirnPin, 1); 
    PWM_out = constrain(PWM_out,26, 229);
    analogWrite(MotorPin, PWM_out); 
  }
}
#endif