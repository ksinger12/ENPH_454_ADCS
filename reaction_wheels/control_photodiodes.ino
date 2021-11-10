// Imports
#include <MsTimer2.h>       // Internal Timer2 for ISR Routine
#include <PinChangeInt.h>  // Create external interrupts
#include <Wire.h>        // IIC communication library

//$$$$$$$$$$$$$$$$$$$ MPU 6050 
MPU6050 mpu6050;     // MPU6050 name mpu6050 
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ; // 6DOF 3-axis acceleration and 3-axis gyroscope variables
//$$$$$$$$$$$$$$$$$$$ MPU 6050 

//************** Maxon motor vars ****************
int enable_pin = 24; //digital input 4: enable
int set_value_pin = 7; //digital input 1: set speed 
int speed_pin = A1; //analog input A1: read averaged speed
//************************************************


//######################## Begin of Kalman Filter Vars 
float Q_angle = 0.001;    // Covariance of gyroscope noise    
float Q_gyro = 0.003;    // Covariance of gyroscope drift noise
float R_angle = 0.5;    // Covariance of accelerometer
char C_0 = 1;
float dt = 0.005; // The value of dt is the filter sampling time (5ms)
float K1 = 0.05; // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate
float K_0,K_1,t_0,t_1;
float angle_err;
float q_bias;    // Gyroscope Drift
float angle;
float angleY_one;
float angle_speed;
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
//########################### End of Kalman Filter Vars 

////////////////////// Begin of PID parameters 
//may need to be tuned for our system
double kp = 28;
double kd = 0.62; 
////////////////////// End of PID parameters 

///////////////////////////////// Begin of PID speed loop Vars 
double kp_speed =  3; 
double ki_speed = 0.072;
double kd_speed = 0; // NOT USED  
double targetAngle = -5; // Angle balance point
int PD_pwm;  //angle output
float pwmOut=0;
float pwmOut2=0; 
///////////////////////////////// End of PID speed loop Vars 


////////////////////// Begin of pulse count /////////////////////////
float avgSpeed;
////////////////////// End of pulse count //////////////////////////

//////////////////////////////// Begin of PI_pwm Vars 
float speeds_filter_old=0;
float positions=0;
double PI_pwm;
int cc;
float speeds_filter;
//////////////////////////////// End of PI_pwm Vars 

void setup()
{
    //set maxon motor control pins
    pinMode(enable_pin, OUTPUT);
    pinMode(set_value_pin, OUTPUT);
    pinMode(speed_pin, INPUT);

    //initial state values
    digitalWrite(enable_pin, HIGH);

    // join I2C bus
    Wire.begin();                            
    delay(500);
    mpu6050.initialize(); //initialize MPU6050
    delay(500);
    Serial.begin(9600);

    //timer for ISR routine
    MsTimer2::set(5, ISR_Routine);
    MsTimer2::start();    //start interrupt

}

void loop() 
{
}

//////////////////// Hall Effect Pulse count ///////////////////////
void getHallSpeed()
{
    //get count from hall sensor (read analog pin)
    avgSpeed = analogRead(speed_pin)
}

///////////////////////////////// Begin of ISR_Routine 
void ISR_Routine()
{
    //This is the main loop of the code (runs repeatedly due to MsTimer)

    sei();  //allow overall interrupt 
    getHallSpeed(); //get hall sensor count in rwPulse

    mpu6050.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);   
    //Get relative angle to light source (TODO: get code from elec team)
    angle_calculate(gyroX);
    PD();         //angle loop PD control
    ReactionWheelPWM(); //actuate motor

    //run PI algorithm every 8th loop to update PI_pwm
    cc++;
    if(cc>=8)     //5*8=40，enter PI algorithm of speed per 40ms
    {
        SpeedPIout();   
        cc=0; 
    }
////////////////////////// End of ISR_Routine 

///////////////////////////// Tilt calculations ///////////////////////
void angle_calculate(gyroX)
{
    //to be provided by elec team (Angle)

    //The X-axis angular velocity calculated by the gyroscope;  the negative sign is the direction processing
    float r = //TODO: add radial length from rotational centre to accelerometer 
    Gyro_x = -gyroX / r;              
    
    //Kalman_Filter would be used here, taking angle_m from elec team, gyro_m from gyroscope
    Kalman_Filter(Angle, Gyro_x);          
}


/////////////////////////////// Kalman Filter Calculations 
//This was used as part of the code to determine the desired angle change for the self-
//balancing wheel, we could use this in a similar way where prior measured
//values are used for the Kalman filter (kalman gain, etc)

//Basically this lets us improve our estimates of the desired angle change by considering
//previously measured values and their errors. 
void Kalman_Filter(double angle_m, double gyro_m)
{
  //Why kalman filter the measured desired angle change and the measured angular speed?
  //Because 

  //Params:
  //    angle_m: Measured desired angle change
  //    gyro_m: Measured system speed with accelerometer
  //updates:
  //    angle_speed (angular speed)
  //    angle (angular displacement relative to desired position)
  angle += gyro_m * dt;          //prior estimate
  angle_err = angle_m - angle; //how does this work?
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //The differential of the covariance of the prior estimate error
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //The integral of the covariance differential of the prior estimate error
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //Intermediate variables in matrix multiplication 
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //denominator
  E = R_angle + C_0 * PCt_0;
  //gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  // Intermediate variables in matrix multiplication
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    // Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    // Posterior estimate
  angle_speed = gyro_m - q_bias;   // The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err; // Posterior estimation; get the optimal angle
}
////////////////////////////////////////////////////////////////

////////////////// Angle PD_pwm ////////////////////
void PD()
{
  //PD angle loop control
  //depends on angle (current angular orientation) and current angular speed
  PD_pwm = kp * angle + kd * angle_speed; 
}

////////////////// Begin of Speed PI_pwm ////////////////////
//Updates PI_pwm (PWM signal from PI part of control algorithm)
void SpeedPIout()
{
  //PI control 
  //Basically update PI_pwm to be 0.7*last hall sensor speed + 0.3*current hall sensor speed

  float speeds = (rwPulse) * 1.0;      //motor speed (from countpulse())
  rwPulse = 0;      
  speeds_filter = speeds_filter_old * 0.7 + speeds * 0.3;
  speeds_filter_old = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -3550,3550);    //Anti-integral saturation
  PI_pwm = ki_speed * (targetAngle - positions) + kp_speed * (targetAngle - speeds_filter);
}
////////////////////////////////////////////////////////////////

//////////////////////////// Final PWM Values 
//I don't understand how this really works, comes from reference code.
//The motor used in the reference code has a brake (that can reduce the 
//angular speed to 0 very quickly). We don't have a brake, so we may not be able to 
//use this code.  
void ReactionWheelPWM()
{
    //Combine the PD and PI parts of the PWM signals into one for actuating

    //TODO: Replace the actuating code
    pwmOut=-PD_pwm; - PI_pwm;           //assign the end value of PWM to motor
    pwmOut = constrain(pwmOut, -255, 255);

    pwmOut2 = map(pwmOut, -255, 255, -180, 130); //TODO modify last 2 values for maxon motor

    //if too far right, PWM to go CW, PWM to go CCW
    if (angle >= 25 && loopOnce == 1) // Reaction Wheel Right Side Jump-up Position 
    {
        //delay before actuating motor
        delay(4000);
        analogWrite(set_value_pin,185);
        delay(4000);
        analogWrite(set_value_pin,10);
        pwmOut2 = 0;
        delay(125);
        loopOnce = 0;
    } 

    //if too far left, delay and run "too far right" code???
    if(angle <= -25 && loopOnce == 0) { // Reaction Wheel Left Side before moving to jump-up position
        //delay before actuating
        delay(2000);
        loopOnce = 1;
    }

    if(angle >= 20 || angle <= -20)  // if angle is greater than +/- 20° motor will stop
        {                                      
            analogWrite(set_value_pin, 0);
        }
        else
        {
            if(pwmOut>=0)         // Reaction wheel leaning to the left from center, so go CW
            {
                analogWrite(set_value_pin, pwmOut2);       
            }
            else // Reaction wheel leaning to the right from center, so go CCW
            {
                analogWrite(set_value_pin, -pwmOut2);  
            }
      } 