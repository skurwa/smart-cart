// master node on i2c network to test floating-point transmission between master and multiple slaves
// by Siddharth Kurwa, 23 September 2018
// https://skurwa.github.io

#include <Wire.h>
#include <PID_v1.h>


// i2c communication
#define PAYLOAD_SIZE 5 // how many bytes to expect from each I2C slave node
#define START_NODE 2 // The starting I2C address of slave nodes

// constants

// left wheel PID parameter containers and values
double measLeftWheel = .01;
double desLeftWheel = .02;
double pwmLeftWheel;
double leftKp = 1;
double leftKi = .01;
double leftKd = .5;

double measRightWheel = .01;
double desRightWheel = .02;
double pwmRightWheel;
double rightKp = 1;
double rightKi = .01;
double rightKd = .5;

byte inPayload[PAYLOAD_SIZE];
byte outPayload[2];

static char doubleStr[15];

void setup()
{
  // start I2C
  Wire.begin();

  // initialize Serial communication for debug
  Serial.begin(9600);
  Serial.println("Master reading");
}

void loop()
{
    // read data from slaves
    Wire.requestFrom(2, PAYLOAD_SIZE);
    if (Wire.available() == PAYLOAD_SIZE) {
        for (int i = 0; i < PAYLOAD_SIZE; i++) {
            inPayload[i] = Wire.read();
        } 
        Serial.println("Payload: ");
        Serial.println(inPayload[0]);
        Serial.println(inPayload[1]);
        Serial.println(inPayload[2]);
        Serial.println(inPayload[3]);
        Serial.println(inPayload[4]);
        measLeftWheel = getWheelVel(inPayload);
    } 
    Serial.print("Wheel velocity: ");
    dtostrf(measLeftWheel,7, 3, doubleStr);
    Serial.println(doubleStr);

    // prepare payload to send to slave
    outPayload[0] = 1;
    double testDouble = 100.3585;
    outPayload[1] = (byte) testDouble; // check if this type conversion works
    Serial.print("outPayload: ");

    Serial.println(outPayload[1]);
    Wire.beginTransmission(2);
    Wire.write(outPayload, 2);
    Wire.endTransmission();

    // loop delay
    delay(1000);
}

    // function to unpack incoming byte array payload into wheel velocity
double getWheelVel(byte Data[5]) {
    // union structure to unpack float data from byte array
    union float2byteArray {
        byte byteArray[4];
        float fval;
    } u;
    u.byteArray[0] = Data[1];
    u.byteArray[1] = Data[2];
    u.byteArray[2] = Data[3];
    u.byteArray[3] = Data[4];
    float wheelVel = u.fval;
    if (Data[0] = 0) {
        wheelVel = -wheelVel;
    }
    return (double) wheelVel;
}