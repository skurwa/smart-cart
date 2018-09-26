#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// pin definition
const int encoderAPin = 2;
const int encoderBPin = 3; // what if I didn't give 2 interrupt pins (worse accuracy but more available)?

//  parameters for velocity calculations
int clicksPerOutRev = 1920; // from Pololu specifications
long sampTime       = 500;  // number of milliseconds to count clicks

// containers
byte pwmWheel   = 0;
float wheelRPM  = 0;
long lastMillis = 0;

// define objects
Encoder enc(encoderAPin, encoderBPin);

void setup() {
    Serial.begin(9600);
    Serial.println("Reading shaft RPM: ");

    // reset time, encoder
    lastMillis = millis();
    enc.write(0);
}

void loop() {
    if ((millis() - lastMillis) > sampTime) {
        // calculate wheel linear velocity (neglecting slip)
        wheelRPM = enc.read() * 60000.00 / (clicksPerOutRev * (millis() - lastMillis));
        // reset time, encoder
        lastMillis = millis();
        enc.write(0);
        Serial.println(wheelRPM);
    }
}