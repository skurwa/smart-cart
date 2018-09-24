#include <Wire.h>
#include <PID_v1.h>


// i2c communication
#define PAYLOAD_SIZE 5 // how many bytes to expect from each I2C slave node
#define START_NODE 2 // The starting I2C address of slave nodes

// constants

// left wheel PID parameter containers and values
float measLeftWheel, desLeftWheel;
int   pwmLeftWheel;
float leftKp = 1;
float leftKi = .01;
float leftKd = .5;

float measRightWheel, desRightWheel;
int   pwmRightWheel,
float rightKp = 1;
float rightKi = .01;
float rightKd = .5;

byte outPayload[2];

// create PID objects
PID leftWheelVelPID(&measLeftWheel, &pwmLeftWheel, &desLeftWheel, leftKp, leftKi, leftKd, DIRECT);
PID rightWheelVelPID(&measRightWheel, &pwmRightWheel, &desRightWheel, rightKp, rightKi, rightKd, DIRECT);

void setup()
{
  // set limits on the PID outputs
  leftWheelVelPID.setOutputLimits(-255,255);
  rightWheelVelPID.setOutputLimits(-255,255);

  // union structure to unpack float data from byte array
  union unpack {
    byte byteArray[4];
    float fval;
  }

  // start I2C
  Wire.begin();

  // initialize Serial communication for debug
  Serial.begin(9600);
  Serial.println("MASTER READER NODE");
  Serial.print("Maximum Slave Nodes: ");
  Serial.println(NODE_MAX);
  Serial.print("Payload size: ");
  Serial.println(PAYLOAD_SIZE);
  Serial.println("***********************");
}

void loop()
{
  leftWheelVelPID.compute();
  rightWheelVelPID.compute();

  // read data from slaves
  Wire.requestFrom(2, PAYLOAD_SIZE);
  if (Wire.available() == PAYLOAD_SIZE) {
    for (int i = 0; i < PAYLOAD_SIZE; i++) Payload[i] = Wire.read();  // get nodes data
    measLeftWheel = getWheelVel(Payload);
    for (int j = 0; j < PAYLOAD_SIZE; j++) Serial.println(Payload[j]);   // print nodes data
  }

  Wire.requestFrom(3, PAYLOAD_SIZE);
  if (Wire.available() == PAYLOAD_SIZE) {
    for (int i = 0; i < PAYLOAD_SIZE; i++) Payload[i] = Wire.read();  // get nodes data
    measRightWheel = getWheelVel(Payload);
    for (int j = 0; j < PAYLOAD_SIZE; j++) Serial.println(Payload[j]);   // print nodes data
  }

  // send updated PWM output to left wheel slave
  if (pwmLeftWheel < 0) {
    pwmLeftWheel = -pwmLeftWheel;
    outPayload[1] = 0; // 0 indicates reverse direction
  }
  else {
    outPayload[1] = 1;
  }
  outPayload[2] = (byte) pwmLeftWheel; // check if this type conversion works

  Wire.beginTransmission(2)
  Wire.write(outPayload, 2)
  Wire.endTransmission();

  if (pwmRightWheel < 0) {
    pwmRightWheel = -pwmRightWheel;
    outPayload[1] = 0;
  }
  else {
    outPayload[1] = 1;
  }
  outPayload[2] = (byte) pwmRightWheel;

  Wire.beginTransmission(3)
  Wire.write(outPayload, 2)
  Wire.endTransmission();

  // loop delay
  delay(100);
}

float getWheelVel(byte Data[5]) {
  unpack wheelVel_union;
  wheelVel_union.byteArray[0] = data[1];
  wheelVel_union.byteArray[1] = data[2];
  wheelVel_union.byteArray[2] = data[3];
  wheelVel_union.byteArray[3] = data[4];
  float wheelVel = wheelVel_union.fval;
  if (Data[0] = 0) {
    wheelVel = -wheelVel;
  }
  return wheelVel;
}
