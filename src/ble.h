#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "gatt_csc.h"

/* CSC simulation configuration */
#define     CSC_FEATURES                            (CSC_FEATURE_WHEEL_REV_DATA | \
                                                    CSC_FEATURE_CRANK_REV_DATA |\
                                                    CSC_FEATURE_MULTIPLE_SENSOR_LOC)


class SkateBLEServer : public BLEServerCallbacks {
    public:
        void setup();
        void loop();
    private:
        bool _hasSpeed = false;

        bool _deviceConnected = false;
        bool _oldDeviceConnected = false;
        unsigned long _simulationUpdateInterval = 1000;
        unsigned long _lastSimulationUpdate = 0;
        unsigned long _updateInterval = 500;
        unsigned long _lastUpdate = 0;

        uint8_t _lastBattery = 80;

        //Cycling Variables
        uint16_t _simulatedSpeedKmph = 30;      
        uint16_t _simulatedCadence = 90;
        bool _increasing = true; 

        //CSC values
        uint32_t _cum_wheel_revs = 0;
        uint16_t _last_wheel_event = 0;
        uint16_t _cum_crank_revs = 0;
        uint16_t _last_crank_event = 0;

        uint8_t _sensorLocation = SENSOR_LOCATION_IN_SHOE;

        byte _cscMeasurement[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        byte _cscFeature[2]      = { 0, 0};

        BLEServer* _pServer;
        BLEService* _pCyclingSpeedAndCadenceService;
        BLEService* _pBatteryLevelService;

        BLECharacteristic* _pCSCMeasurementCharacteristic;
        BLECharacteristic* _pSensorLocationCharacteristic;
        BLECharacteristic* _pSCControlPointCharacteristic;
        BLECharacteristic* _pCSCFeatureCharacteristic;
        BLECharacteristic* _pBatteryLevelCharacteristic;

        void updateClient();
        void updateSpeedAndCadence(uint16_t speedKmph, uint16_t cadenceRPM);

        void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
            auto remoteAddress = getRemoteAddress(param);
            Serial.println("Client connected: " + String(remoteAddress));
            _deviceConnected = true;

        };
        void onDisconnect(BLEServer* pServer) {
            Serial.println("Client disconnected");
            _deviceConnected = false;
        }

        String getRemoteAddress(esp_ble_gatts_cb_param_t* param);
        void updateSpeedAndCadence();
};

void SkateBLEServer::setup() {
    Serial.println("Starting BLE Server");
    BLEDevice::init("Skate-O-Meter");
    _pServer = BLEDevice::createServer();
    _pServer->setCallbacks(this);

    if (_hasSpeed) {
        _cscMeasurement[0] = 0b00000011;
        _cscFeature[0] = 0b00000011;
    } else {
        _cscMeasurement[0] = 0b00000010;
        _cscFeature[0] = 0b00000010;
    }

    _pCyclingSpeedAndCadenceService = _pServer->createService(GATT_CSC_SERVICE_UUID);
    _pCSCMeasurementCharacteristic = _pCyclingSpeedAndCadenceService->createCharacteristic(GATT_CSC_MEASUREMENT_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    _pCSCMeasurementCharacteristic->addDescriptor(new BLE2902());
    _pSensorLocationCharacteristic = _pCyclingSpeedAndCadenceService->createCharacteristic(GATT_SENSOR_LOCATION_UUID, BLECharacteristic::PROPERTY_READ);
    _pSensorLocationCharacteristic->setValue(&_sensorLocation, 1);
    _pSCControlPointCharacteristic = _pCyclingSpeedAndCadenceService->createCharacteristic(GATT_SC_CONTROL_POINT_UUID, BLECharacteristic::PROPERTY_INDICATE);
    _pCSCFeatureCharacteristic     = _pCyclingSpeedAndCadenceService->createCharacteristic(GATT_CSC_FEATURE_UUID, BLECharacteristic::PROPERTY_INDICATE);
    _pCSCFeatureCharacteristic->setValue(_cscFeature, sizeof(_cscFeature));
    _pCyclingSpeedAndCadenceService->start();

    _pBatteryLevelService = _pServer->createService(GATT_BATTERY_SERVICE_UUID);
    _pBatteryLevelCharacteristic = _pBatteryLevelService->createCharacteristic(GATT_BATTERYLEVEL_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));
    BatteryLevelDescriptor.setValue("Percentage 0 - 100");
    _pBatteryLevelCharacteristic->addDescriptor(&BatteryLevelDescriptor);
    _pBatteryLevelCharacteristic->addDescriptor(new BLE2902());
    _pBatteryLevelService->start();

    auto pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(GATT_CSC_SERVICE_UUID);
    BLEDevice::startAdvertising();
    Serial.println("BLE Server started. Now you can read it in your phone.");
}

inline void SkateBLEServer::loop() {
    updateSpeedAndCadence();
    if (millis() - _lastSimulationUpdate >= _simulationUpdateInterval) {
        _lastSimulationUpdate = millis();
        if (_increasing) {
            _simulatedSpeedKmph++;
            _simulatedCadence++;
            _lastBattery--;
        } else {
            _simulatedSpeedKmph--;
            _simulatedCadence--;
            _lastBattery++;
        }

        if (_simulatedSpeedKmph >= 50)
            _increasing = false;   
        if (_simulatedSpeedKmph <= 20)
            _increasing = true;

        Serial.print("Increasing: " + String(_increasing));
        Serial.print(", Speed: " + String(_simulatedSpeedKmph));
        Serial.print("km/h, Cadence: " + String(_simulatedCadence));
        Serial.println("rpm, Battery level "+String(_lastBattery)+String("%"));
    }

    // disconnecting
    if (!_deviceConnected && _oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        BLEDevice::startAdvertising();   
        Serial.println("Start advertising");
        _oldDeviceConnected = _deviceConnected;
    }
    // connecting
    if (_deviceConnected && !_oldDeviceConnected) {
        Serial.println("Connecting...");
		// do stuff here on connecting
        _oldDeviceConnected = _deviceConnected;
    }
}

inline void SkateBLEServer::updateClient() {
    if (!_deviceConnected)
        return;

    _pBatteryLevelCharacteristic->setValue(&_lastBattery, 1);
    _pBatteryLevelCharacteristic->notify();

    //The Cycle Measurement Characteristic data is defined here:
    //https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.csc_measurement.xml
    //Frist byte is flags. Wheel Revolution Data Present (0 = false, 1 = true) = 1, Crank Revolution Data Present (0 = false, 1 = true), so the flags are 0x03 (binary 11 converted to HEX).
    //buf[0]=0x03;
    
    // Setting values for cycling measures (from the Characteristic File)
    //Cumulative Wheel Revolutions (unitless)
    //Last Wheel Event Time (Unit has a resolution of 1/1024s)
    //Cumulative Crank Revolutions (unitless)
    //Last Crank Event Time (Unit has a resolution of 1/1024s)
    
    if (_cscMeasurement[0] & CSC_FEATURE_WHEEL_REV_DATA) {
        memcpy(&_cscMeasurement[1], &_cum_wheel_revs, 4);
        memcpy(&_cscMeasurement[5], &_last_wheel_event, 2);
        memcpy(&_cscMeasurement[7], &_cum_crank_revs, 2);
        memcpy(&_cscMeasurement[9], &_last_crank_event, 2);
    } else {
        memcpy(&_cscMeasurement[1], &_cum_crank_revs, 2);
        memcpy(&_cscMeasurement[3], &_last_crank_event, 2);
    }

    _pCSCMeasurementCharacteristic->setValue(_cscMeasurement, sizeof(_cscMeasurement));
    _pCSCMeasurementCharacteristic->notify();
    // Serial.println("- - - - - - - - - - - - - - - - - - - - - - -");
    // Serial.println("Battery level "+String(_lastBattery));
    // Serial.println("Cum Wheel Rev : " + String(_cum_wheel_revs));
    // Serial.println("Cum Crank : " + String(_cum_crank_revs));
    // Serial.println("Last Wheel Event " + String(_last_wheel_event));
    // Serial.println("Last Crank Event " + String(_last_crank_event));
}

inline String SkateBLEServer::getRemoteAddress(esp_ble_gatts_cb_param_t* param)
{
    char remoteAddress[18];
    sprintf(
        remoteAddress,
        "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
        param->connect.remote_bda[0],
        param->connect.remote_bda[1],
        param->connect.remote_bda[2],
        param->connect.remote_bda[3],
        param->connect.remote_bda[4],
        param->connect.remote_bda[5]
    );
    return String(remoteAddress);
}

//---------------------------------------------------------------------------------------------
    /* Update simulated CSC measurements.
     * Each call increments wheel and crank revolution counters by one and
     * computes last event time in order to match simulated candence and speed.
     * Last event time is expressedd in 1/1024th of second units.
     *
     *                 60 * 1024
     * crank_dt =    --------------
     *                cadence[RPM]
     *
     *
     *                circumference[mm] * 1024 * 60 * 60
     * wheel_dt =    -------------------------------------
     *                         10^6 * speed [kph] 
     */
     //---------------------------------------------------------------------------------------------
inline void SkateBLEServer::updateSpeedAndCadence()
{
    static double wheelRevolutions=0;
    static double crankRevolutions=0;
    static uint32_t cum_wheel_revs_old=0;
    static uint16_t cum_crank_revs_old=0;

    static unsigned long lastMillis;
    unsigned long millisPassed = millis() - lastMillis; // time difference between last call
    lastMillis = millis();
    bool shouldUpdateClient = false;

    /* Calculate simulated measurement values */
    if (_simulatedSpeedKmph > 0) {
        double speedmps = _simulatedSpeedKmph/3.6;
        double mmTravelled = speedmps * millisPassed;
        wheelRevolutions += mmTravelled/WHEEL_CIRCUMFERENCE_MM;
        _cum_wheel_revs = wheelRevolutions;
        if (_cum_wheel_revs != cum_wheel_revs_old) {
            _last_wheel_event += 1024 * ((_cum_wheel_revs - cum_wheel_revs_old) * WHEEL_CIRCUMFERENCE_MM)/(speedmps*1000); // re-calculation of timestamp for the int value
            cum_wheel_revs_old = _cum_wheel_revs;
            shouldUpdateClient = true;
        }
    }
    
    if (_simulatedCadence > 0) {
        double cadenceS = _simulatedCadence/60.0;
        crankRevolutions += cadenceS * (millisPassed/1000.0);
        _cum_crank_revs = crankRevolutions;
        if (_cum_crank_revs != cum_crank_revs_old) {
            _last_crank_event += 1024 * (_cum_crank_revs - cum_crank_revs_old)/cadenceS;
            cum_crank_revs_old = _cum_crank_revs;
            shouldUpdateClient = true;
        }
    }

    if (shouldUpdateClient) {
        updateClient();
    }
}
