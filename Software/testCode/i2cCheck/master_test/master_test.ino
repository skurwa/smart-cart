// master node on i2c network to test floating-point transmission between master and multiple slaves
// by Siddharth Kurwa, 23 September 2018
// https://skurwa.github.io

#include <Wire.h>
#define PAYLOAD_SIZE 1 // 1 byte initial payload

byte outPayload = 0;

void setup {
    Serial.begin(9600);
    Serial.println("Master Reader: ");
    Wire.begin();
}

void loop {
    // send data to slave
    Wire.beginTransmission(2);
    Wire.write(outPayload)
    Wire.endTransmission();

    outPayload++;

    // request data from slave
    Wire.requestFrom(1, PAYLOAD_SIZE);
    if (Wire.available() == PAYLOAD_SIZE) {
        int payload;
        for (int i = 0; i < PAYLOAD_SIZE, i++) {
            payload = Wire.read();
            Serial.print("Data from slave: ");
            Serial.println(payload);
        }
    }

    // reset byte to avoid overflow
    if (outPayload == 255) {
        outPayload = 0;
    }

}

