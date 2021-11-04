

// Imports
#include <MsTimer2.h>       // Internal Timer2 for ISR Routine
#include <PinChangeInt.h>  // Create external interrupts
#include <MPU6050.h>      // MPU6050 library
#include <Wire.h>        // IIC communication library


//$$$$$$$$$$$$$$$$$$$ MPU 6050 
MPU6050 mpu6050;     // MPU6050 name mpu6050 
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ; // 6DOF 3-axis acceleration and 3-axis gyroscope variables
//$$$$$$$$$$$$$$$$$$$ MPU 6050 

//************** Maxon motor vars ****************
int enable_pin = 24; //digital input 4: enable
int set_value_pin = 7; //digital input 1: set speed 
//************************************************

//@@@@@@@@@@@@@@@@@@@@@@@ Begin of Angle Vars @@@@@@@@@@@@@@@@@@@@@@@@@@
float angle0 = -5; //mechanical balance angle (ideally 0 degrees) 
float Gyro_x;
float Gyro_y;
float Gyro_z;  //Angular velocity by gyroscope calculation
//@@@@@@@@@@@@@@@@@@@@@@@@ End of Angle Vars @@@@@@@@@@@@@@@@@@@@@@@@@@@

//######################## Begin of Kalman Filter Vars 
float Q_angle = 0.001;    // Covariance of gyroscope noise    
float Q_gyro = 0.003;    // Covariance of gyroscope drift noise
float R_angle = 0.5;    // Covariance of accelerometer
char C_0 = 1;
float dt = 0.005; // The value of dt is the filter sampling time
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
double kp = 28;
double ki = 0; // NOT USED
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

////////////////// Begin of ISR speed count vars
#define nidecHalleffect 12  // External interrupt D12 for Nidec FG output 6 Pulses/Revoltion
volatile long countHall = 0;// Used to calculate the pulse value calculated by the Hall encoder 
////////////////// End of ISR speed count vars

////////////////////// Begin of pulse count /////////////////////////
int rw = 0;
int pulseCount = 0;
int rwPulse;
////////////////////// End of pulse count //////////////////////////

//////////////////////////////// Begin of PI_pwm Vars 
float speeds_filterold=0;
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

    //initial state values
    digitalWrite(enable_pin, HIGH);

    //join I2C bus for MPU6050
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
    //I don't understand what this is, attachPinChangeInterrupt is never defined
    //external interrupt; used to calculate the reaction wheel's speed
    attachPinChangeInterrupt(nidecHalleffect, Code_Hall, CHANGE);  
}

///////////////////// Hall Effect count /////////////////////////
// encoder count
void Code_Hall() 
{
  countHall ++; //increment hall pulse count
} 

//////////////////// Hall Effect Pulse count ///////////////////////
void countpulse()
{
    //get count from hall sensor (TODO)

}

///////////////////////////////// Begin of ISR_Routine 
void ISR_Routine()
{
    sei();  //allow overall interrupt 
    countpulse(); //get hall sensor count in rwPulse

    mpu6050.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);     
    //IIC to get MPU6050 six-axis data  ax ay az gx gy gz
    angle_calculate(accX, accY, accZ, gyroX, gyroY, gyroZ, dt, Q_angle, 
        Q_gyro, R_angle, C_0, K1);
    PD();         //angle loop PD control
    ReactionWheelPWM(); //actuate motor

    //run PI algorithm every 8th loop
    cc++;
    if(cc>=8)     //5*8=40，enter PI algorithm of speed per 40ms
    {
        SpeedPIout();   
        cc=0; 
    }
////////////////////////// End of ISR_Routine 

///////////////////////////// Tilt calculations ///////////////////////
void angle_calculate(int16_t accX,int16_t accY,int16_t accZ,int16_t 
    gyroX,int16_t gyroY,int16_t gyyroZ,float dt,float Q_angle,float 
    Q_gyro,float R_angle,float C_0,float K1)
{
    float Angle = -atan2(accY , accZ) * (180/ PI);           //Radial rotation angle calculation formula ; negative sign is direction processing
    Gyro_x = -gyroX / 131;              //The X-axis angular velocity calculated by the gyroscope;  the negative sign is the direction processing
    Kalman_Filter(Angle, Gyro_x);            //Kalman Filter
    //rotating angle Z-axis parameter
    Gyro_z = -gyroZ / 131;                      //angle speed of Z-axis
    //accelz = accZ / 1604;

    float angleAx = -atan2(accX, accZ) * (180 / PI); //calculate the inclined angle with x-axis
    Gyro_y = -gyroY / 131.00; //angle speed of Y-axis
    Yiorderfilter(angleAx, Gyro_y); //first-order filtering
}


/////////////////////////////// Kalman Filter Calculations 
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;          //prior estimate
  angle_err = angle_m - angle;
  
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

///////////////////// First-order filter /////////////////
void Yiorderfilter(float angle_m, float gyro_m)
{
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

////////////////// Angle PD_pwm ////////////////////
void PD()
{
  PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD angle loop control
}

////////////////// Begin of Speed PI_pwm ////////////////////
void SpeedPIout()
{
  float speeds = (rwPulse) * 1.0;      //motor speed (from countpulse())
  rwPulse = 0;
  speeds_filterold *= 0.7;         //first-order complementary filtering
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -3550,3550);    //Anti-integral saturation
  PI_pwm = ki_speed * (targetAngle - positions) + kp_speed * (targetAngle - speeds_filter); //speed loop control PI
}
////////////////////////////////////////////////////////////////

//////////////////////////// Final PWM Values 
void ReactionWheelPWM()
{
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