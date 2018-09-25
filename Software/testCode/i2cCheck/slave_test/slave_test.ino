// slave 1  node on i2c network to test floating-point comm  between master and multiple slaves
// by Siddharth Kurwa, 23 September 2018
// https://skurwa.github.io

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>

// define parameters for velocity calculations
float wheelRadiusFeet = 0.25; // in feet
int clicksPerOutputRev = 1920;
long  samplingFreq = 1000000; // number of microseconds to wait for clicks
byte dir;
byte pwmWheel;
float wheelVel;

// i2c communication
int address =  2;  // address of I2C slave
int outPayloadSize = 5; // bytes to be received by the master I2C node
volatile byte* wheelVelPtr;

// define pins
const int encoderAPin = 2;
const int encoderBPin = 3;
const int motorDirPin = 4;
const int motorPWMPin = 5;

long lastMicros;
byte outPayload[5];


// define library objects
Encoder enc(encoderAPin, encoderBPin);

void setup()
{
  // initialize pins
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  // start I2C communication
  Wire.begin(address);  // Activate I2C network
  Wire.onRequest(requestEvent); // Request attention of master node
  Wire.onReceive(receiveEvent);

  lastMicros = micros();
}

void loop()
{
  // process encoder input
  // if ((micros() - lastMicros) > samplingFreq) {
  //   float wheelRPM = 500 * 1000000.00 / (clicksPerOutputRev * (micros() - lastMicros)) + enc.read(); // rev per sec
  //   wheelVel = wheelRadiusFeet * wheelRPM;
  //   enc.write(0);
  //   lastMicros = micros();
  // }
  wheelVel = 123.456789;

  // write outputs to motor
  digitalWrite(motorDirPin, dir);
  digitalWrite(motorPWMPin, pwmWheel) ;
}

void requestEvent()
{
    // prepare payload
    union float2byteArray {
        byte byteArray[4];
        float floatVal;
    } u;
    u.floatVal = wheelVel;
    
    outPayload[0] = dir; // DOUBLEC CHECK THIS IF YOU HAVE ISSUES (the commanded direction is being used instead of the measured direction)
    outPayload[1] = u.byteArray[0];
    outPayload[2] = u.byteArray[1];
    outPayload[3] = u.byteArray[2];
    outPayload[4] = u.byteArray[3];

    // send payload to master
    Wire.write(outPayload, outPayloadSize);
}

// receive payload from master
void receiveEvent(int byteCount) {
  dir = Wire.read();
  pwmWheel = Wire.read();
}
