#include "remote_control.h"
#include "earthSeeking.h"

const int IR_RECEIVE_PIN = 2;

int MODE = 0; 
int BUFFER_ANGLE = 0; 
int TARGET_ANGLE = 0; 

//desired angle change
double changeAngle;

void setup(){
  Serial.begin(9600);
  irrecv.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), readIRRemote, CHANGE); 
}

void loop(){
  // Note that sun seeking & earth seeking is really a special case of target angle=0
  // which means we can probably get rid of the explicit sun/earth seeking mode
  // and replace it with sun/earth angle mode where angle=0. 

    switch(MODE){
      case SUN_SEEKING_MODE:
        Serial.println("MODE: SUN SEEKING"); 
        // setup pid method
        while(MODE==SUN_SEEKING_MODE) {
          delay(100); // run pid method
        }
        break;
        
      case EARTH_SEEKING_MODE:
        Serial.println("MODE: EARTH SEEKING"); 
        // setup pid method
        setupEarthSeeking(TARGET_ANGLE)
        while(MODE==EARTH_SEEKING_MODE) {
          readBdirection();
          changeAngle = getEarthAngle();
          updatePID(changeAngle);
          actuateMotor();
          delay(5); // run pid method
        }
        break;
        
      case CALIBRATE_SUN_SENSOR_MODE:
        Serial.println("MODE: CALIBRATE SUN SENSORS"); 
        // run calibration method
        MODE = IDLE_MODE; 
        break;
        
      case SUN_ANGLE_MODE:
        Serial.print("MODE: SUN ANGLE. TARGET ANGLE: "); 
        Serial.println(TARGET_ANGLE);
        // setup pid method
        while(MODE==SUN_ANGLE_MODE) {
          delay(100); // run pid method
        }
        BUFFER_ANGLE = 0;
        break;
        
      case EARTH_ANGLE_MODE:
        Serial.print("MODE: EARTH ANGLE. TARGET ANGLE: "); 
        Serial.println(TARGET_ANGLE);
        // setup pid method
        while(MODE==EARTH_ANGLE_MODE) {
          delay(100); // run pid method
        }
        BUFFER_ANGLE = 0;
        break;

        
      case DE_TUMBLE_MODE:
        Serial.println("MODE: DE-TUMBLE"); 
        // setup de-tumble method
        while(MODE==DE_TUMBLE_MODE) {
          delay(100); // run de-tumble method
        }
        break;
    }
}
