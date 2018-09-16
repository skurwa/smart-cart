#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>

// define parameters for velocity calculations
float wheelRadiusFeet = 0.25; // in feet
int clicksPerOutputRev = 1920;
int samplingFreq = 1000; // number of microseconds to wait for clicks
byte dir;
byte pwmWheel;
float wheelVel;

// i2c communication
int address 2;  // address of I2C slave
int payloadSize 5; // bytes to be received by the master I2C node
volatile byte* wheelVelPtr;

// define pins
const int encoderAPin;
const int encoderBPin;
const int motorDirPin;
const int motorPWMPin;

// define library objects
Encoder enc(encoderAPin, encoderBPin);

void setup()
{
  // initialize pins
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);

  // testing I2C communication through serial port
  Serial.begin(9600);
  Serial.println("SLAVE SENDER NODE");
  Serial.print("Node address: ");
  Serial.println(address);
  Serial.print("Payload size: ");
  Serial.println(payloadSize);
  Serial.println("***********************");

  // start I2C communication
  Wire.begin(address);  // Activate I2C network
  Wire.onRequest(requestEvent); // Request attention of master node
  Wire.onReceive(receiveEvent);

  lastMicros = micros();
}

void loop()
{
  // process encoder input
  if (micros() - lastMicros) > samplingFreq) {
    float wheelRPM = enc.read() * 1000000.00 / (clicksPerOutputRev * (micros() - lastMicros)); // rev per sec
    wheelVel = wheelRadius * wheelRPM;
    enc.write(0);
    lastMicros = micros();
  }

  // write outputs to motor
  digitalWrite(motorDirPin, dir);
  digitalWrite(motorVelPin, pwmWheel) ;
}

void requestEvent()
{
  // prepare payload
  byte* Payload;
  wheelVelPtr = (byte*) &wheelVel;
  Payload[0] = dir; // DOUBLEC CHECK THIS IF YOU HAVE ISSUES (the commanded direction is being used instead of the measured direction)
  Payload[1] = wheelVelPtr[0];
  Payload[2] = wheelVelPtr[1];
  Payload[3] = wheelVelPtr[2];
  Payload[4] = wheelVelPtr[3];

  // send payload to master
  Wire.write(Payload, payloadSize);
}

// receive payload from master
void receiveEvent(int byteCount) {
  dir = Wire.read();
  pwmWheel = Wire.read();
}
