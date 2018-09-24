// slave 1  node on i2c network to test floating-point comm  between master and multiple slaves
// by Siddharth Kurwa, 23 September 2018
// https://skurwa.github.io

#include <Wire.h>

byte outPayload;

void setup {
    Wire.begin(2);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
}

void loop {
    delay(100);
}

void requestEvent() {
    Wire.write(outPayload, 1);
}

void receiveEvent(int byteCount) {
    outPayload = Wire.read();
}