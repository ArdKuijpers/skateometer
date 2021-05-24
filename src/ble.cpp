#include "ble.h"

auto hrmUUID = BLEUUID((uint16_t)0x180D);
auto batteryUUID = BLEUUID((uint16_t)0x180F);
auto cyclingSpeedCadenceUUID = BLEUUID((uint16_t)0x1816);

void SkateBLEServer::setup() {


    Serial.print("Starting BLE Server");
    BLEDevice::init("SkateOMeter");
    Serial.print("...SkateOMeter...");

    switch (_type) {
        case BLEServerType::hrm:
            _bleAppearance = ESP_BLE_APPEARANCE_HEART_RATE_BELT;
            break;
        case BLEServerType::speedcadence:
            _featureFlags = 0b00000011;
            _bleAppearance = ESP_BLE_APPEARANCE_CYCLING_SPEED_CADENCE;
            break;
        case BLEServerType::cadence:
            _featureFlags = 0b00000010;
            _bleAppearance = ESP_BLE_APPEARANCE_CYCLING_CADENCE;
            break;
    }

    auto pServer = BLEDevice::createServer();
    pServer->setCallbacks(this);
    Serial.print("...services...");

    setupBatteryService(pServer);
    setupDeviceInfoService(pServer);

    if (_bleAppearance == ESP_BLE_APPEARANCE_HEART_RATE_BELT)
        setupHRMService(pServer);
    else 
        setupCyclingSpeedCadenceService(pServer);
    
    Serial.print("...advertising...");
    setupAdvertising();
    Serial.println("done");
}

void SkateBLEServer::loop() {
    static bool increasing = true;
    static bool oldDeviceConnected = false;

    updateCSCMeasurements();
    if (millis() - _lastSimulationUpdate >= _simulationUpdateInterval) {
        _lastSimulationUpdate = millis();
        if (increasing) {
            _simulatedSpeed++;
            _simulatedCadence++;
            _simulatedBattery--;
            _simulatedHeartRate+=4;
        } else {
            _simulatedSpeed--;
            _simulatedCadence--;
            _simulatedBattery++;
            _simulatedHeartRate-=4;
        }

        if (_simulatedSpeed >= 50)
            increasing = false;   
        if (_simulatedSpeed <= 20)
            increasing = true;

        Serial.print("Speed: " + String(_simulatedSpeed)+String(" km/h, "));
        Serial.print("Cadence: " + String(_simulatedCadence)+String(" rpm, "));
        Serial.print("HR: "+String(_simulatedHeartRate)+String(" BPM, "));
        Serial.println("Battery level "+String(_simulatedBattery)+String("%"));
        notifyBattery();
        notifyHRM();
    }

    // disconnecting
    if (!_deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        BLEDevice::startAdvertising();   
        Serial.println("Start advertising");
        oldDeviceConnected = _deviceConnected;
    }
    // connecting
    if (_deviceConnected && !oldDeviceConnected) {
        Serial.println("Connecting...");
		// do stuff here on connecting
        oldDeviceConnected = _deviceConnected;
    }
}

void SkateBLEServer::setupUnknownGarminService(BLEServer* pServer)
{
    auto service = pServer->createService("6a4e2401-667b-11e3-949a-0800200c9a66");
    auto pCharacteristic = service->createCharacteristic("6a4ecd28-667b-11e3-949a-0800200c9a66", BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->addDescriptor(new BLE2902());
    
    service->createCharacteristic("6a4e4c80-667b-11e3-949a-0800200c9a66", BLECharacteristic::PROPERTY_WRITE_NR);
    service->start();
}

void SkateBLEServer::setupCyclingSpeedCadenceService(BLEServer* pServer)
{
    auto service = pServer->createService(cyclingSpeedCadenceUUID);
    auto pCSCFeatureCharacteristic = service->createCharacteristic(BLEUUID((uint16_t)0x2A5C), BLECharacteristic::PROPERTY_READ);
    auto pCSCFeatureDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    pCSCFeatureDescriptor->setValue("CSC Feature");
    pCSCFeatureCharacteristic->addDescriptor(pCSCFeatureDescriptor);
    byte cscFeatureFlags[2] = {_featureFlags, 0};
    pCSCFeatureCharacteristic->setValue(cscFeatureFlags, 2);

    _pCSCMeasurementCharacteristic = service->createCharacteristic(BLEUUID((uint16_t)0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);
    _pCSCMeasurementCharacteristic->addDescriptor(new BLE2902());
    // auto pValue = new esp_attr_value_t();
    // auto datagramsize = 11;
    // auto pExampleValue = new byte[datagramsize];
    // pExampleValue[0] = _featureFlags;
    // pValue->attr_len = datagramsize;
    // pValue->attr_max_len = datagramsize;
    // pValue->attr_value = pExampleValue;
    // _pCSCMeasurementCharacteristic->setCharacteristicValue(pValue);

    auto pSensorLocationCharacteristic = service->createCharacteristic((uint16_t)0x2A5D, BLECharacteristic::PROPERTY_READ);
    /* Sensor location enum 
     * other                   0
     * top_of_shoe             1
     * in_shoe                 2
     * hip                     3
     * front_wheel             4
     * left_crank              5
     * right_crank             6
     * left_pedal              7
     * right_pedal             8
     * frot_hub                9
     * rear_dropout            10
     * chainstay               11
     * rear_wheel              12
     * rear_hub                13
     * chest                   14
     * spider                  15
     * chain_ring              16
     */
    byte sensorLocationValue[1] = {0};
    pSensorLocationCharacteristic->setValue(sensorLocationValue, 1);

    auto pSCControlPointCharacteristic = service->createCharacteristic(BLEUUID((uint16_t)0x2A55), BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE);
    pSCControlPointCharacteristic->addDescriptor(new BLE2902());
    service->start();
}

void SkateBLEServer::setupHRMService(BLEServer* pServer)
{
    auto service = pServer->createService(hrmUUID);
    _pHeartRateMeasurementCharacteristic = service->createCharacteristic(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
    auto pDescriptor = new BLE2902();
    pDescriptor->setNotifications(true);
    _pHeartRateMeasurementCharacteristic->addDescriptor(pDescriptor);
   
    /* Body Sensor Location
     * 0x00 Other
     * 0x01 Chest
     * 0x02 Wrist
     * 0x03 Finger
     * 0x04 Hand
     * 0x05 Ear Lobe
     * 0x06 Foot
     * 0x07–0xFF Reserved for Future Use
    */
    byte sensorLocationValue[1] = { 0x01 }; // Chest
    auto pSensorLocationCharacteristic = service->createCharacteristic((uint16_t)0x2A38, BLECharacteristic::PROPERTY_READ);
    pSensorLocationCharacteristic->setValue(sensorLocationValue, 1);
    service->start();
}

void SkateBLEServer::setupBatteryService(BLEServer* pServer)
{
    auto service = pServer->createService(batteryUUID);
    _pBatteryLevelCharacteristic = service->createCharacteristic(BLEUUID((uint16_t)0x2A19), BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));
    BatteryLevelDescriptor.setValue("Percentage 0 - 100");
    _pBatteryLevelCharacteristic->addDescriptor(&BatteryLevelDescriptor);
    _pBatteryLevelCharacteristic->addDescriptor(new BLE2902());
    service->start();
}

void SkateBLEServer::setupDeviceInfoService(BLEServer *pServer)
{
    BLEService* service = pServer->createService(BLEUUID((uint16_t)0x180A));
    BLECharacteristic *manufacturer = service->createCharacteristic(BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ);
    manufacturer->setValue("Etxean");
    BLECharacteristic *modelnumber = service->createCharacteristic(BLEUUID((uint16_t)0x2A24), BLECharacteristic::PROPERTY_READ);
    modelnumber->setValue("666");
    BLECharacteristic *hardware = service->createCharacteristic(BLEUUID((uint16_t)0x2A27), BLECharacteristic::PROPERTY_READ);
    hardware->setValue("1.0.0");
    BLECharacteristic *firmware = service->createCharacteristic(BLEUUID((uint16_t)0x2A26), BLECharacteristic::PROPERTY_READ);
    firmware->setValue("1.0.1");
    BLECharacteristic *software = service->createCharacteristic(BLEUUID((uint16_t)0x2A28), BLECharacteristic::PROPERTY_READ);
    software->setValue("1.0.1");
    service->start();
}

void SkateBLEServer::notifyBattery() {
    if (!_deviceConnected)
        return;

    Serial.println("Battery: " + String(_simulatedBattery)+String("%"));
    _pBatteryLevelCharacteristic->setValue(&_simulatedBattery, 1);
    _pBatteryLevelCharacteristic->notify();
}

void SkateBLEServer::notifyCyclingSpeedCadence() {
    if (!_deviceConnected || (_type != BLEServerType::speedcadence && _type != BLEServerType::cadence))
        return;

    //The Cycle Measurement Characteristic data is defined here:
    //https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.csc_measurement.xml

    byte cscMeasurement[11] = {_featureFlags, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int cadenceOffset = 0;
    if (hasSpeed()) {
        memcpy(&cscMeasurement[1], &_cum_wheel_revs, 4);
        memcpy(&cscMeasurement[5], &_last_wheel_event, 2);
        Serial.print("Wheel - rev: " + String(_cum_wheel_revs));
        Serial.println(", event time: " + String(_last_wheel_event));
        cadenceOffset=6;
    }
    memcpy(&cscMeasurement[1+cadenceOffset], &_cum_crank_revs, 2);
    memcpy(&cscMeasurement[3+cadenceOffset], &_last_crank_event, 2);
    Serial.print("Crank - rev: " + String(_cum_crank_revs));
    Serial.println(", event time: " + String(_last_crank_event));
    _pCSCMeasurementCharacteristic->setValue(cscMeasurement, 5+cadenceOffset);
    _pCSCMeasurementCharacteristic->notify();
}

void SkateBLEServer::notifyHRM() {
    if (!_deviceConnected || _bleAppearance != ESP_BLE_APPEARANCE_HEART_RATE_BELT)
        return;

    /*
    Field #1 - Flags (byte)
        Bit 0   - Heart Rate Value Format
                    0 = uint8
                    1 = uint16
        Bit 1-2 - Sensor Contact Status
                    0 - Sensor Contact feature is not supported in the current connection
                    1 - Sensor Contact feature is not supported in the current connection
                    2 - Sensor Contact feature is supported, but contact is not detected
                    3 - Sensor Contact feature is supported and contact is detected
        Bit 3   - Energy Expended Status
                    0 = Energy Expended field is not present
                    1 = Energy Expended field is present. Units: kilo Joules
        Bit 3   - RR-Interval bit
                    0 = RR-Interval values are not present.
                    1 = One or more RR-Interval values are present.
        Bit 5-7 - Reserved
    Field #2 - Heart Rate Measurement Value (uint8)
    Field #3 - Heart Rate Measurement Value (uint16)
    Field #4 - Energy Expended (uint16)
    Field #5 - RR-Interval (uint16)
    */

    // // Flags + Heart rate (uint16) = 3 bytes
    // byte hrmMeasurement[3] = { 0b00000001, 0, 0 };
    // memcpy(&hrmMeasurement[1], &_simulatedHRM, 2);
    //_pHeartRateMeasurementCharacteristic->setValue(value, 3);

    // // Flags + Heart rate (unit8) = 2 bytes
    byte hrmMeasurement[2] = { 0b00000000, 0 };
    hrmMeasurement[1] = _simulatedHeartRate;
    Serial.println("HR: " + String(_simulatedHeartRate)+String(" BPM"));
    _pHeartRateMeasurementCharacteristic->setValue(hrmMeasurement, 2);
    _pHeartRateMeasurementCharacteristic->notify();
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

#define	WHEEL_CIRCUMFERENCE_MM 2105
void SkateBLEServer::updateCSCMeasurements()
{
    static double wheelRevolutions=0;
    static double crankRevolutions=0;
    static uint32_t cum_wheel_revs_old=0;
    static uint16_t cum_crank_revs_old=0;

    static unsigned long lastMillis;
    unsigned long millisPassed = millis() - lastMillis; // time difference between last call
    lastMillis = millis();
    bool shouldNotify = false;

    /* Calculate simulated measurement values */
    if (hasSpeed() && _simulatedSpeed > 0) {
        double speedmps = _simulatedSpeed/3.6;
        double mmTravelled = speedmps * millisPassed;
        wheelRevolutions += mmTravelled/WHEEL_CIRCUMFERENCE_MM;
        _cum_wheel_revs = wheelRevolutions;
        if (_cum_wheel_revs != cum_wheel_revs_old) {
            _last_wheel_event += (1024.0 * (_cum_wheel_revs - cum_wheel_revs_old) * WHEEL_CIRCUMFERENCE_MM)/(speedmps*1000); // re-calculation of timestamp for the int value
            cum_wheel_revs_old = _cum_wheel_revs;
            shouldNotify = true;
        }
    }
    
    if (_simulatedCadence > 0) {
        double cadenceS = _simulatedCadence/60.0;
        crankRevolutions += cadenceS * (millisPassed/1000.0);
        _cum_crank_revs = crankRevolutions;
        if (_cum_crank_revs != cum_crank_revs_old) {
            _last_crank_event += 1024 * (_cum_crank_revs - cum_crank_revs_old)/cadenceS;
            cum_crank_revs_old = _cum_crank_revs;
            shouldNotify = true;
        }
    }

    if (shouldNotify) {
        notifyCyclingSpeedCadence();
    }
}

void SkateBLEServer::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
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
    Serial.println("Client connected: " + String(remoteAddress));
    _deviceConnected = true;

};

void SkateBLEServer::onDisconnect(BLEServer* pServer) {
    Serial.println("Client disconnected");
    _deviceConnected = false;
}

void SkateBLEServer::setupAdvertising() {
    auto pAdvertising = BLEDevice::getAdvertising();
    ::esp_ble_gap_config_local_icon(_bleAppearance);
    pAdvertising->setAppearance(_bleAppearance);

    if (_type == BLEServerType::hrm)
        pAdvertising->addServiceUUID(hrmUUID);
    else
        pAdvertising->addServiceUUID(cyclingSpeedCadenceUUID);
    
    // pAdvertising->setScanResponse(true);
    // pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    // pAdvertising->setMinPreferred(0x12);
    // pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    // pAdvertising->setScanResponse(false);
    BLEDevice::startAdvertising();
}