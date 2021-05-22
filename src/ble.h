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
        bool _deviceConnected = false;
        bool _oldDeviceConnected = false;
        unsigned long _updateInterval = 1000;
        unsigned long _lastUpdate = 0;

        uint8_t _lastBattery = 100;

        //Cycling Variables
        uint16_t _simulatedSpeedKmph = 0;      
        uint16_t _simulatedCrankRpm = 0; 

        uint16_t _wheel_rev_period = 0;
        uint16_t _crank_rev_period = 0;
        uint32_t _cum_wheel_rev = 10;
        uint16_t _last_wheel_event = 0;
        uint16_t _cum_cranks = 0;
        uint16_t _last_crank_event = 0;

        uint8_t _sensorLocation = SENSOR_LOCATION_IN_SHOE;

        byte _cscMeasurement[11] = { 0b00000010, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        byte _cscFeature[2]      = { 0b00000010, 0};

        BLEServer* _pServer;
        BLEService* _pCyclingSpeedAndCadenceService;
        BLEService* _pBatteryLevelService;

        BLECharacteristic* _pCSCMeasurementCharacteristic;
        BLECharacteristic* _pSensorLocationCharacteristic;
        BLECharacteristic* _pSCControlPointCharacteristic;
        BLECharacteristic* _pCSCFeatureCharacteristic;
        BLECharacteristic* _pBatteryLevelCharacteristic;


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
        void simulate_speed_and_cadence();
};

void SkateBLEServer::setup() {
    Serial.println("Starting BLE Server");
    BLEDevice::init("Skate-O-Meter");
    _pServer = BLEDevice::createServer();
    _pServer->setCallbacks(this);
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
    if (millis() - _lastUpdate >= _updateInterval) {
        _lastUpdate = millis();
        simulate_speed_and_cadence();
        if (--_lastBattery < 50) {
            _lastBattery = 100;
        }
        
        if (_deviceConnected) {
            _pBatteryLevelCharacteristic->setValue(&_lastBattery, 1);
            _pBatteryLevelCharacteristic->notify();
            Serial.println("---------------------------------------------");
            Serial.println("Battery level "+String(_lastBattery)+String("% posted"));

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
                memcpy(&_cscMeasurement[1], &_cum_wheel_rev, 4);
                memcpy(&_cscMeasurement[5], &_last_wheel_event, 2);
                memcpy(&_cscMeasurement[7], &_cum_cranks, 2);
                memcpy(&_cscMeasurement[9], &_last_crank_event, 2);
            } else {
                memcpy(&_cscMeasurement[1], &_cum_cranks, 2);
                memcpy(&_cscMeasurement[3], &_last_crank_event, 2);
            }

            _pCSCMeasurementCharacteristic->setValue(_cscMeasurement, sizeof(_cscMeasurement));
            _pCSCMeasurementCharacteristic->notify();
            Serial.println("---------------------------------------------");
            Serial.println("Speed (km/h) : " + String(_simulatedSpeedKmph));
            Serial.println("Crank RPM : " + String(_simulatedCrankRpm));
            Serial.println("Wheel Rev Period " + String(_wheel_rev_period));
            Serial.println("Crank Rev Period " + String(_crank_rev_period));
            Serial.println("- - - - - - - - - - - - - - - - - - - - - - -");
            Serial.println("Cum Wheel Rev : " + String(_cum_wheel_rev));
            Serial.println("Cum Crank : " + String(_cum_cranks));
            Serial.println("Last Wheel Event " + String(_last_wheel_event));
            Serial.println("Last Crank Event " + String(_last_crank_event));
        }
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
inline void SkateBLEServer::simulate_speed_and_cadence()
{
    /* Update simulated crank and wheel rotation speed */
    _simulatedSpeedKmph++;
    if (_simulatedSpeedKmph >= MAX_SPEED_KPH) {
         _simulatedSpeedKmph = MIN_SPEED_KPH;
    }
    
    _simulatedCrankRpm++;
    if (_simulatedCrankRpm >= MAX_CRANK_RPM) {
         _simulatedCrankRpm = MIN_CRANK_RPM;
    }
    
    /* Calculate simulated measurement values */
    if (_simulatedSpeedKmph > 0){
        _wheel_rev_period = (36*64*WHEEL_CIRCUMFERENCE_MM) / 
                           (625*_simulatedSpeedKmph);
        _cum_wheel_rev++;
        _last_wheel_event += _wheel_rev_period;
    }
    
    if (_simulatedCrankRpm > 0){
        _crank_rev_period = (60*1024) / _simulatedCrankRpm;
        _cum_cranks++;
        _last_crank_event += _crank_rev_period; 
    }
}


