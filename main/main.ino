//installed / built in packages
#include "Adafruit_MLX90393.h"
#include <SoftwareSerial.h>
#include <IRremote.h>

//custom package imports
#include <SqrtFit.h>
#include <LightVectorDetermination.h>
#include "remote_control.h"
#include "earthSeeking.h"
#include "sunSeeking.h"
#include "pid_control.h"


// Sun Sensor calibration stuff
const int n_cal_readings = 64;        // number of readings/motor steps used for calibration
const int n_photodiode = 4;           // number of photodiodes
const int photodiode_pin_offset = 0;  // Note: we assume photodiode pin order is sequential - this is the starting pin number
const float n_average = 30.0;         // num samples to average over per reading
bool insert_header = true;
LightVectorDetermination LVD = LightVectorDetermination(n_photodiode, n_cal_readings, photodiode_pin_offset, n_average);

// Bluetooth initialization
SoftwareSerial bt(2,3); // RX, TX

// Gryoscope initialization
MPU6050 mpu;

// Magnetometer initialization
Adafruit_MLX90393 sensor = Adafruit_MLX90393();

//pins for motor control
const int enablePin = 8;
const int MotorPin = 11;
const int dirnPin = 10;

//PID set point, input, output
double Setpoint = 0;
double PWM_out;
double angle;

//PID gains
const double kp = 0.75;
const double ki = 0.002;
const double kd = 10;

//for manual PID implementation in sun seeking and earth seeking
double rateError;
double cumError, lastError;
unsigned long currentTime, previousTime, elapsedTime;
double error;

// Remote and Control
const int IR_RECEIVE_PIN = 18;
int MODE = 0;
int BUFFER_ANGLE = 0;
int TARGET_ANGLE = 0;

// Magnetometer stuff
double changeAngle;

void setup(){
    //bluetooth shield
    bt.begin(9600);
    irrecv.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
    attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), readIRRemote, CHANGE);
    //gyroscope
    mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
    mpu.calibrateGyro();
    //magnetometer
    if (! sensor.begin_I2C()) {         
    bt.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
    }
    else{
        bt.println("Found a MLX90393 sensor");
    }

    sensor.setGain(MLX90393_GAIN_2_5X);
    sensor.setResolution(MLX90393_X, MLX90393_RES_19);
    sensor.setResolution(MLX90393_Y, MLX90393_RES_19);
    sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
    sensor.setOversampling(MLX90393_OSR_2);
    sensor.setFilter(MLX90393_FILTER_6);
}

void loop(){
    switch(MODE){
        case SUN_SEEKING_MODE:
            bt.println("SUN SEEKING MODE. TARGET ANGLE: ");
            bt.println(TARGET_ANGLE);
            // setup pid method
            setupPID(TARGET_ANGLE);
            while(MODE==SUN_SEEKING_MODE) {
                bt.println(LVD.get_global_angle());
                getSunAngle();
                updatePID();
                actuateMotor();
                delay(5); // run pid method
            }
            break;

        case EARTH_SEEKING_MODE:
            bt.println("EARTH SEEKING MODE. TARGET ANGLE: ");
            bt.println(TARGET_ANGLE);
            // setup pid method
            setupPID(TARGET_ANGLE);
            while(MODE==EARTH_SEEKING_MODE) {
                readBdirection();
                getEarthAngle();
                updatePID();
                actuateMotor();
                delay(5); // run pid method
            }
            break;

        case CALIBRATE_SUN_SENSOR_MODE:
            bt.println("CALIBRATE SUN SENSORS MODE");
            LVD.auto_calibrate();
            MODE = IDLE_MODE;
            break;

        case DE_TUMBLE_MODE:
            bt.println("MODE: DE-TUMBLE");
            // setup de-tumble method
            while(MODE==DE_TUMBLE_MODE) {
                delay(100); // run de-tumble method
            }
            break;
    }
}
