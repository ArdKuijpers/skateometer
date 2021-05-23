#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

enum BLEServerType { hrm, cadence, speedcadence };

class SkateBLEServer : public BLEServerCallbacks {
    public:
        SkateBLEServer(BLEServerType type = BLEServerType::speedcadence) : _type(type) {}
        void setup();
        void loop();
    private:
        BLEServerType _type;
        bool _deviceConnected = false;
        unsigned long _simulationUpdateInterval = 1000;
        unsigned long _lastSimulationUpdate = 0;

        // Speed in km/h
        uint16_t _simulatedSpeed = 30;      
        // Cadence in RPM
        uint16_t _simulatedCadence = 90;
        // Battery level [0-100%]
        uint8_t _simulatedBattery = 80;
        // Simulated HRM
        uint8_t _simulatedHeartRate = 100;

        //Cumulative Wheel Revolutions (unitless)
        uint32_t _cum_wheel_revs = 0;
        //Last Wheel Event Time (Unit has a resolution of 1/1024s)
        uint16_t _last_wheel_event = 0;
        //Cumulative Crank Revolutions (unitless)
        uint16_t _cum_crank_revs = 0;
        //Last Crank Event Time (Unit has a resolution of 1/1024s)
        uint16_t _last_crank_event = 0;

        // Byte flag indicating capabilities of the Cylcing Speed Cadence Sensor
        // https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.csc_measurement.xml
        byte _featureFlags;

        uint16_t _bleAppearance;
        void setupBatteryService(BLEServer* pServer);
        
        BLECharacteristic* _pCSCMeasurementCharacteristic;
        void notifyBattery();
        void setupCyclingSpeedCadenceService(BLEServer* pServer);
        void notifyCyclingSpeedCadence();
        bool hasSpeed() { return _featureFlags & 0b00000001; }

        BLECharacteristic* _pHeartRateMeasurementCharacteristic;
        void setupHRMService(BLEServer *pServer);
        void notifyHRM();

        BLECharacteristic* _pBatteryLevelCharacteristic;
        void setupDeviceInfoService(BLEServer *pServer);

        void setupAdvertising();
        void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param);
        void onDisconnect(BLEServer* pServer);
        void updateCSCMeasurements();
};
