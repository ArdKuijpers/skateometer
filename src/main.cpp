#include <Arduino.h>
#include "mpu.h"
#include "ble.h"

SkateMPU smpu;
SkateBLEServer bleserver(BLEServerType::cadence);

void setup() {
    Serial.begin(115200);
    //smpu.setup();
    bleserver.setup();
}

void loop() {
    //smpu.loop();
    bleserver.loop();
}

