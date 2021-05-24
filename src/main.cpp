#include <Arduino.h>
#include "mpu.h"
#include "ble.h"
#include "heartbeater.h"

const int heartbeatPin = 16;

SkateMPU smpu;
SkateBLEServer bleserver(BLEServerType::speedcadence);
Heartbeater heartbeater(heartbeatPin, LOW);

void setup() {
    Serial.begin(115200);
    //smpu.setup();
    bleserver.setup();
}

void loop() {
    //smpu.loop();
    bleserver.loop();
    heartbeater.loop();
}

