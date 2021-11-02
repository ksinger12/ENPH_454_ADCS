
#define NUMBER_OF_TURNS = ;
#define AREA_OF_COILS = ;
float currentThroughCoils[3]; // 3 column array (3D vector -> x,y,z) -> one for each solenoid
float angularVelocityMotor[3];
float angularVelocitySatellite[3];
float k = ; // going to need a relative magnitude value of k -> further research may be need or just example math
float kCurrent = ; //current used to calculate k
float magneticField[3]; // earth's magnetic field -> 3 coumn array (3D vector -> x,y,z)
bool movement; // how are we moving -> detumbling satellite vs. detumbling motors | true = motors, false = satellite

void setup() {
  // put your setup code here, to run once:
  //define initial values of variables (that are not constants)
  movement = true; //detumbling motors
}

void loop() { //start w/ x,y,z -> assuming all in same reference frame
  angularMomentumMotor[2] = ; // input value read from hall sensor on reaction wheel
  angularMomentumSatellite[2] = {}; //input value from external hall sensor
  magneticField = {}; //input from magnetometer

  if (movement) {
    current = currentCalculator(magneticField, angularMomentumMotor);
  } else {
    current = currentCalculator(magneticField, angularMomentumSatellite);
  }

}

void currentCalculator(float magneticField[], float angularVelocity[]) {
  float constant = k*(NUMBER_OF_TURNS*AREA_OF_COILS);
  float newCurrent[] = {constant*magneticField[1]*angularVelocity[2], constant*magneticField[0]*angularVelocity[2]};
  
  if (newCurrent[0]+newCurrent[1] > __) { //use current that calculated k with
    float magnitude = sqrt(pow(newCurrent[0],2) + pow(newCurrent[1], 2))
    newCurrent[0] = newCurrent[0]/magnitude*kCurrent;
    newCurrent[1] = newCurrent[1]/magnitude*kCurrent;
  }
  return newCurrent;
}


  // B is in Tesla -> 40000 nT = 4e4e-9 = 4e-5
  // angular velocity is rad/s -> say 0.1 rad/s -> 1e-1
  // current = k*cross(w,B)/(n*A) -> in amps ~= 40 mA = 4e-2
  // therefore say k = 67200

  //saturation check 
  //if the sum of the absolute value of the current is greate than 0.04
  // normalize it: current = current/normal(current)*0.04

  //now we have current -> which needs to be calculated everytime
  // magnetic dipole is then: muB = current*n*A 

  //that is how we detumble
  
  
}
