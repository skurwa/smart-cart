#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>

#define OUT_PAYLOAD_SIZE  5
#define ADDRESS           3

// pin definition
const int encoderAPin = 2;
const int encoderBPin = 3;
const int motorDirPin = 4;
const int motorPWMPin = 5;

//  parameters for velocity calculations
int clicksPerOutRev = 1920; // from Pololu specifications
long sampTime       = 300;  // number of milliseconds to count clicks

// containers
int dir         = 0;
int pwmWheel    = 0;
float wheelRPM  = 0;
long lastMillis = 0;

// define objects
Encoder enc(encoderAPin, encoderBPin);

void setup() {
  // initialize pins
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  // start I2C communication
  Wire.begin(ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // log time at start of first loop
  lastMillis = millis();
  enc.write(0);
}

void loop() {
  // process encoder input
  if ((millis() - lastMillis) > sampTime) {
    // calculate wheel angular velocity (neglecting slip)
    wheelRPM = enc.read() * 60000.00 / (clicksPerOutRev * (millis() - lastMillis));

    // reset time, encoder
    lastMillis = millis();
    enc.write(0);
  }

  // write outputs to motor
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, pwmWheel);

  // loop delay
  delay(50);
}

// when master asks for data
void requestEvent() {
  // get direction
  byte measDir;
  byte OutPayload[OUT_PAYLOAD_SIZE];
  if (wheelRPM < 0) {
    wheelRPM      = -wheelRPM;
    OutPayload[0] = 0;
  }
  else {
    OutPayload[0] = 1;
  }

  // union structure to convert floating point wheel velocity to byte array
  union float2ByteArray {
      byte ByteArray[4];
      float floatVal;
  } u;
  u.floatVal = wheelRPM;
  OutPayload[1] = u.ByteArray[0];
  OutPayload[2] = u.ByteArray[1];
  OutPayload[3] = u.ByteArray[2];
  OutPayload[4] = u.ByteArray[3];

  // send payload to master
  Wire.write(OutPayload, OUT_PAYLOAD_SIZE);
}

// when master wants to give data
void receiveEvent(int byteCount) {
  dir      =  Wire.read(); // first byte is direction
  pwmWheel =  Wire.read(); // second byte is PWM
}