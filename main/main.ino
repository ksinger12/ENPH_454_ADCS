#include <IRremote.h>
#include <SqrtFit.h>
#include <LightVectorDetermination.h>
#include "remote_control.h"
#include "earthSeeking.h"
//#include "sunSeeking.h"
#include <SoftwareSerial.h>

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

// Remote and Control
const int IR_RECEIVE_PIN = 18;
int MODE = 0;
int BUFFER_ANGLE = 0;
int TARGET_ANGLE = 0;

// Magnetometer stuff
double changeAngle;

void setup(){
    bt.begin(9600);
    irrecv.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
    attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), readIRRemote, CHANGE);
    mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
    mpu.calibrateGyro();
}

void loop(){
    switch(MODE){
        case SUN_SEEKING_MODE:
            bt.println("SUN SEEKING MODE. TARGET ANGLE: ");
            bt.println(TARGET_ANGLE);
            // setup pid method
            while(MODE==SUN_SEEKING_MODE) {
                bt.println(LVD.get_global_angle());
                delay(100); // run pid method
            }
            break;

        case EARTH_SEEKING_MODE:
            bt.println("EARTH SEEKING MODE. TARGET ANGLE: ");
            bt.println(TARGET_ANGLE);
            // setup pid method
            setupEarthSeeking(TARGET_ANGLE);
            while(MODE==EARTH_SEEKING_MODE) {
                readBdirection();
                changeAngle = getEarthAngle();
                updatePID(changeAngle);
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