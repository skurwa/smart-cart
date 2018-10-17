#include <Wire.h>
#include <PID_v1.h>

// i2c communication
#define  IN_PAYLOAD_SIZE     5 // incoming bytes from slaves
#define  OUT_PAYLOAD_SIZE    2 // outgoing bytes to slaves
#define  LEFT_WHEEL_ADDRESS  2 // address of left wheel  
#define  RIGHT_WHEEL_ADDRESS 3 // address of right wheel

// containers
char desiredOutputSwitch = 'f';
long lastChange = 0;
// PID
double  measLeftWheel, desLeftWheel, pwmLeftWheel;
double leftKp = .9;
double leftKi = .6;
double leftKd = 0;
double measRightWheel, desRightWheel, pwmRightWheel;
double rightKp = 1;
double rightKi = .01;
double rightKd = .5;
PID leftWheelAngVelPID(&measLeftWheel, &pwmLeftWheel, &desLeftWheel, leftKp, leftKi, leftKd, DIRECT);
PID rightWheelAngVelPID(&measRightWheel, &pwmRightWheel, &desRightWheel, rightKp, rightKi, rightKd, DIRECT);

void setup() {
  // for debug
  Serial.begin(9600);

  // set limits on the PID outputs
  leftWheelAngVelPID.SetOutputLimits(-255,255);
  rightWheelAngVelPID.SetOutputLimits(-255,255);

  // start I2C
  Wire.begin();

  leftWheelAngVelPID.SetMode(AUTOMATIC);
  // rightWheelAngVelPID.SetMode(AUTOMATIC);
}

void loop() {
  if ((millis() - lastChange > 10000) && (millis() - lastChange < 20000)) {
    desiredOutputSwitch = 'p';
  }
  if ((millis() - lastChange > 20000) && (millis() - lastChange < 30000)) {
    desiredOutputSwitch = 'f';
  }
  if ((millis() - lastChange > 30000) && (millis() - lastChange < 40000)) {
    desiredOutputSwitch = 'b';
  }
  if ((millis() - lastChange > 40000) && (millis() - lastChange < 50000)) {
    desiredOutputSwitch = 's';
  }
  if (millis() - lastChange > 50000) {
    desiredOutputSwitch = 'p';
  }
  
  // for debug
  if (Serial.available() > 0) {
    desiredOutputSwitch = Serial.read();
  }
  // Serial.print("Drive state: ");
  // Serial.println(desiredOutputSwitch);
  // Serial.print("Desired wheel velocity: ");
  // Serial.println(desLeftWheel);
  // Serial.print("Measured wheel velocity: ");
  Serial.println(measLeftWheel);
  // Serial.print("PWM command: ");
  // Serial.println(pwmLeftWheel);

  
  // update wheel speeds from slave controllers;
  getWheelAngVel(LEFT_WHEEL_ADDRESS, measLeftWheel);
  // getWheelAngVel(RIGHT_WHEEL_ADDRESS, measRightWheel);

  // state machine to quickly move between drive states
  switch(desiredOutputSwitch) {
    case 'f': // straight-line forward drive
      desLeftWheel  = 250;
      desRightWheel = 250;
      break;
    case 's': // slow straight-line forward drive
      desLeftWheel  = 125;
      desRightWheel = 125;
      break;
    case 'l': // turn left
      desLeftWheel  = 0;
      desRightWheel = 100;
      break;
    case 'r': // turn right
      desLeftWheel  = 100;
      desRightWheel = 0;
      break;
    case 'b': // slow straight-line back up 
      desLeftWheel  = -125;
      desRightWheel = -125;
      break;
    case 'p': // pause
      desLeftWheel  = 0;
      desRightWheel = 0;
      break;
    default: // default state is paused
      desRightWheel = 250;
      desLeftWheel  = 250;
  }

  // compute outputs based on updated measured and desired angular velocities
  leftWheelAngVelPID.Compute();
  // rightWheelAngVelPID.Compute();

  // send motor outputs to slave controllers;
  giveMotorOutput(LEFT_WHEEL_ADDRESS, pwmLeftWheel);
  // giveMotorOutput(RIGHT_WHEEL_ADDRESS, pwmRightWheel);

  // loop delay
  delay(25);
}

void getMotor 


// request and process wheel angular velocity data from slave controllers
void getWheelAngVel(int address, double &wheelAngVel) {
  // local container to store incoming payload
  byte InPayload[IN_PAYLOAD_SIZE];

  // i2c comm to get payload from slave controller
  Wire.requestFrom(address, IN_PAYLOAD_SIZE);
  if (Wire.available() == IN_PAYLOAD_SIZE) {
    // get  wheel speed
    for (int i = 0; i < IN_PAYLOAD_SIZE; i++) {
      InPayload[i] = Wire.read();
    }
    wheelAngVel = unpackWheelAngVel(InPayload);
  }
}

// provide updated PWM and direction outputs to slave controllers
void giveMotorOutput(int address, double output) {
  // local container to package outgoing payload
  byte outPayload[OUT_PAYLOAD_SIZE];

  // reconfigure output to be packaged into byte payload
  if (output < 0) {
    output = -output;
    outPayload[0] = 0; // 0 indicates reverse direction
  }
  else {
    outPayload[0] = 1;
  }
  outPayload[1] = (byte) output;

  // send payload to slave controller
  Wire.beginTransmission(address);
  Wire.write(outPayload, OUT_PAYLOAD_SIZE);
  Wire.endTransmission();
}

// process raw payload into wheel angular velocity
double unpackWheelAngVel(byte Data[5]) {
  // union structure to unpack byte array into float
  union float2byteArray {
      byte byteArray[4];
      float fval;
  } u;
  u.byteArray[0] = Data[1];
  u.byteArray[1] = Data[2];
  u.byteArray[2] = Data[3];
  u.byteArray[3] = Data[4];
  float wheelAngVel = u.fval;

  // assign directionality
  if (Data[0] == 0) {
      wheelAngVel = -1 * wheelAngVel;
  }
  return (double) wheelAngVel;
}