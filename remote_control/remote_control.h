//
// Created by Andrew Hayman on 2021-11-24.
//

#ifndef ENPH_454_ADCS_REMOTE_CONTROL_H
#define ENPH_454_ADCS_REMOTE_CONTROL_H

#define IDLE_MODE 0
#define SUN_SEEKING_MODE 1
#define EARTH_SEEKING_MODE 2
#define CALIBRATE_SUN_SENSOR_MODE 3
#define DE_TUMBLE_MODE 6

#include <Arduino.h>
#include <IRremote.h>

extern int MODE;
extern int BUFFER_ANGLE;
extern int TARGET_ANGLE;
extern const int IR_RECEIVE_PIN;

IRrecv irrecv(IR_RECEIVE_PIN);

void readIRRemote() {
    while (irrecv.decode()){
        //Serial.println(irrecv.decodedIRData.decodedRawData, HEX); // For finding button addresses
        switch(irrecv.decodedIRData.decodedRawData) {
            // 0
            case 0xE916FF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+0;
                break;
                // 1
            case 0xF30CFF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+1;
                break;
                // 2
            case 0xE718FF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+2;
                break;
                // 3
            case 0xA15EFF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+3;
                break;
                // 4
            case 0xF708FF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+4;
                break;
                // 5
            case 0xE31CFF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+5;
                break;
                // 6
            case 0xA55AFF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+6;
                break;
                // 7
            case 0xBD42FF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+7;
                break;
                // 8
            case 0xAD52FF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+8;
                break;
                // 9
            case 0xB54AFF00:
                BUFFER_ANGLE = BUFFER_ANGLE*10+9;
                break;

                // PLAY
            case 0xEA15FF00:
                BUFFER_ANGLE = 0;
                MODE = CALIBRATE_SUN_SENSOR_MODE;
                break;

                // BACK
            case 0xF807FF00:
                break;

                // FORWARD
            case 0xF609FF00:
                break;

                // PLUS
            case 0xBF40FF00:
                TARGET_ANGLE = BUFFER_ANGLE;
                BUFFER_ANGLE= 0;
                MODE = SUN_SEEKING_MODE;
                break;

                // MINUS
            case 0xE619FF00:
                TARGET_ANGLE = BUFFER_ANGLE;
                BUFFER_ANGLE = 0;
                MODE = EARTH_SEEKING_MODE;
                break;

                // C
            case 0xF20DFF00:
                BUFFER_ANGLE = 0;
                break;
                // TEST
            case 0xBB44FF00:
                break;
                // UNDO
            case 0xBC43FF00:
                break;

                // POWER
            case 0xBA45FF00:
                BUFFER_ANGLE = 0;
                MODE = IDLE_MODE;
                bt.println("IDLE MODE.");
                break;
                // MENU
            case 0xB847FF00:
                BUFFER_ANGLE = 0;
                MODE = DE_TUMBLE_MODE;
                break;
        }
        irrecv.resume();
    }
}

void printMode() {
    switch(MODE){
        case 0:
            Serial.println("MODE: IDLE");
            break;
        case 1:
            Serial.println("MODE: SUN SEEKING");
            break;
        case 2:
            Serial.println("MODE: EARTH SEEKING");
            break;
        case 3:
            Serial.println("MODE: CALIBRATE SUN SENSORS");
            break;
        case 4:
            Serial.print("MODE: SUN ANGLE. TARGET ANGLE: ");
            Serial.println(TARGET_ANGLE);
            break;
        case 5:
            Serial.print("MODE: EARTH ANGLE. TARGET ANGLE: ");
            Serial.println(TARGET_ANGLE);
            break;
        case 6:
            Serial.print("MODE: DE-TUMBLE");
            break;
    }
}
#endif //ENPH_454_ADCS_REMOTE_CONTROL_H
