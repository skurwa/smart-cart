#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>

#define OUT_PAYLOAD_SIZE  5
#define ADDRESS           3

// pin definition
const int encoderAPin = 2;
const int encoderBPin = 3; // what if I didn't give 2 interrupt pins (worse accuracy but more available)?
const int motorDirPin = 4;
const int motorPWMPin = 5;

//  parameters for velocity calculations
float wheelRadius   = 0.25;   // in feet
int clicksPerOutRev = 1920;   // from Pololu specifications
long sampTime       = 500000; // number of microseconds to count clicks

// containers
byte OutPayload[OUT_PAYLOAD_SIZE];
byte dir        = 1;
byte pwmWheel   = 0;
float wheelVel  = 0;
long lastMicros = 0;

// define objects
Encoder enc(encoderAPin, encoderBPin);

void setup()
{
  // initialize pins
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  // start I2C communication
  Wire.begin(ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // log time at start of first loop
  lastMicros = micros();
}

void loop()
{
  // process encoder input
  if ((micros() - lastMicros) > sampTime) {
    // calculate wheel linear velocity (neglecting slip)
    wheelVel = enc.read() * 1000000 * wheelRadius / (clicksPerOutRev * (micros() - lastMicros));

    // reset time, encoder
    lastMicros = micros();
    enc.write(0);
  }

  // write outputs to motor
  digitalWrite(motorDirPin, dir);
  digitalWrite(motorPWMPin, pwmWheel) ;
}

// when master asks for data
void requestEvent() {
  // union structure to convert float to byte array
  union float2ByteArray {
      byte ByteArray[4];
      float floatVal;
  } u;
  u.floatVal = wheelVel;

  // prepare payload for transmission
  OutPayload[0] = dir; // DOUBLEC CHECK THIS IF YOU HAVE ISSUES (the commanded direction is being used instead of a measured direction)
  OutPayload[1] = u.ByteArray[0];
  OutPayload[2] = u.ByteArray[1];
  OutPayload[3] = u.ByteArray[2];
  OutPayload[4] = u.ByteArray[3];

  // send payload to master
  Wire.write(OutPayload, OUT_PAYLOAD_SIZE);
}

// when master wants to give data
void receiveEvent(int byteCount) {
  dir =       Wire.read(); // first byte is direction
  pwmWheel =  Wire.read(); // second byte is PWM
}
