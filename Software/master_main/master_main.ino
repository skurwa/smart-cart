#include <Wire.h>
#include <PID_v1.h>

// i2c communication
#define IN_PAYLOAD_SIZE 5   // incoming bytes from slaves
#define OUT_PAYLOAD_SIZE 2  // outgoing bytes to slaves

// containers
byte InPayload[IN_PAYLOAD_SIZE];
byte outPayload[OUT_PAYLOAD_SIZE];

// PID
double  measLeftWheel, desLeftWheel, pwmLeftWheel;
double leftKp = 1;
double leftKi = .01;
double leftKd = .5;
double measRightWheel, desRightWheel, pwmRightWheel;
double rightKp = 1;
double rightKi = .01;
double rightKd = .5;
PID leftWheelVelPID(&measLeftWheel, &pwmLeftWheel, &desLeftWheel, leftKp, leftKi, leftKd, DIRECT);
PID rightWheelVelPID(&measRightWheel, &pwmRightWheel, &desRightWheel, rightKp, rightKi, rightKd, DIRECT);

void setup()
{
  // set limits on the PID outputs
  leftWheelVelPID.SetOutputLimits(-255,255);
  rightWheelVelPID.SetOutputLimits(-255,255);

  // start I2C
  Wire.begin();
}

void loop()
{
  leftWheelVelPID.Compute();
  rightWheelVelPID.Compute();

  // request left wheel speed from controller
  Wire.requestFrom(2, IN_PAYLOAD_SIZE);
  if (Wire.available() == IN_PAYLOAD_SIZE) {
    // get left wheel speed
    for (int i = 0; i < IN_PAYLOAD_SIZE; i++) InPayload[i] = Wire.read();
    measLeftWheel = getWheelVel(InPayload);
  }

  // request right wheel speed from controller
  Wire.requestFrom(3, IN_PAYLOAD_SIZE);
  if (Wire.available() == IN_PAYLOAD_SIZE) {
    // get right wheel speed
    for (int i = 0; i < IN_PAYLOAD_SIZE; i++) InPayload[i] = Wire.read();  
    measRightWheel = getWheelVel(InPayload);
  }

  // send updated PWM to left wheel controller
  if (pwmLeftWheel < 0) {
    pwmLeftWheel = -pwmLeftWheel;
    outPayload[1] = 0; // 0 indicates reverse direction
  }
  else {
    outPayload[1] = 1;
  }
  outPayload[2] = (byte) pwmLeftWheel; // check if this type conversion works

  Wire.beginTransmission(2);
  Wire.write(outPayload, OUT_PAYLOAD_SIZE);
  Wire.endTransmission();

  // send updated PWM to right wheel controller 
  if (pwmRightWheel < 0) {
    pwmRightWheel = -pwmRightWheel;
    outPayload[1] = 0;
  }
  else {
    outPayload[1] = 1;
  }
  outPayload[2] = (byte) pwmRightWheel;

  Wire.beginTransmission(3);
  Wire.write(outPayload, OUT_PAYLOAD_SIZE);
  Wire.endTransmission();

  // loop delay
  delay(500);
}

// function to get wheel velocity from incoming payload
double getWheelVel(byte Data[5]) {
    // union structure to unpack byte array into float
    union float2byteArray {
        byte byteArray[4];
        float fval;
    } u;
    u.byteArray[0] = Data[1];
    u.byteArray[1] = Data[2];
    u.byteArray[2] = Data[3];
    u.byteArray[3] = Data[4];
    float wheelVel = u.fval;

    // assign directionality
    if (Data[0] = 0) {
        wheelVel = -wheelVel;
    }
    return (double) wheelVel;
}