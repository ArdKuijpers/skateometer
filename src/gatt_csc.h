
//global definitions
#define 	SPEED_AND_CADENCE_MEAS_INTERVAL         1000
#define 	WHEEL_CIRCUMFERENCE_MM                  2105
#define 	KPH_TO_MM_PER_SEC                       278
#define 	MIN_SPEED_KPH                           1
#define 	MAX_SPEED_KPH                           40
#define 	SPEED_KPH_INCREMENT                     1
#define 	DEGREES_PER_REVOLUTION                  360
#define 	RPM_TO_DEGREES_PER_SEC                  6
#define 	MIN_CRANK_RPM                           20
#define 	MAX_CRANK_RPM                           100
#define 	CRANK_RPM_INCREMENT                     3

/* Cycling Speed and Cadence configuration */
#define     GATT_CSC_SERVICE_UUID                   BLEUUID((uint16_t)0x1816)
#define     GATT_CSC_MEASUREMENT_UUID               BLEUUID((uint16_t)0x2A5B)
#define     GATT_CSC_FEATURE_UUID                   BLEUUID((uint16_t)0x2A5C)
#define     GATT_SENSOR_LOCATION_UUID               BLEUUID((uint16_t)0x2A5D)
#define     GATT_SC_CONTROL_POINT_UUID              BLEUUID((uint16_t)0x2A55)

/* Device Information configuration */
#define     GATT_DEVICE_INFO_UUID                   BLEUUID((uint16_t)0x180A)
#define     GATT_MANUFACTURER_NAME_UUID             BLEUUID((uint16_t)0x2A29)
#define     GATT_MODEL_NUMBER_UUID                  BLEUUID((uint16_t)0x2A24)

/*CSC Measurement flags*/
#define     CSC_MEASUREMENT_WHEEL_REV_PRESENT       0x01
#define     CSC_MEASUREMENT_CRANK_REV_PRESENT       0x02

/* CSC feature flags */
#define     CSC_FEATURE_WHEEL_REV_DATA              0x01
#define     CSC_FEATURE_CRANK_REV_DATA              0x02
#define     CSC_FEATURE_MULTIPLE_SENSOR_LOC         0x04

/* Sensor location enum */
#define     SENSOR_LOCATION_OTHER                   0
#define     SENSOR_LOCATION_TOP_OF_SHOE             1
#define     SENSOR_LOCATION_IN_SHOE                 2
#define     SENSOR_LOCATION_HIP                     3
#define     SENSOR_LOCATION_FRONT_WHEEL             4
#define     SENSOR_LOCATION_LEFT_CRANK              5
#define     SENSOR_LOCATION_RIGHT_CRANK             6
#define     SENSOR_LOCATION_LEFT_PEDAL              7
#define     SENSOR_LOCATION_RIGHT_PEDAL             8
#define     SENSOR_LOCATION_FROT_HUB                9
#define     SENSOR_LOCATION_REAR_DROPOUT            10
#define     SENSOR_LOCATION_CHAINSTAY               11
#define     SENSOR_LOCATION_REAR_WHEEL              12
#define     SENSOR_LOCATION_REAR_HUB                13
#define     SENSOR_LOCATION_CHEST                   14
#define     SENSOR_LOCATION_SPIDER                  15
#define     SENSOR_LOCATION_CHAIN_RING              16

/* SC Control Point op codes */
#define     SC_CP_OP_SET_CUMULATIVE_VALUE           1
#define     SC_CP_OP_START_SENSOR_CALIBRATION       2
#define     SC_CP_OP_UPDATE_SENSOR_LOCATION         3
#define     SC_CP_OP_REQ_SUPPORTED_SENSOR_LOCATIONS 4
#define     SC_CP_OP_RESPONSE                       16

/*SC Control Point response values */
#define     SC_CP_RESPONSE_SUCCESS                  1
#define     SC_CP_RESPONSE_OP_NOT_SUPPORTED         2
#define     SC_CP_RESPONSE_INVALID_PARAM            3
#define     SC_CP_RESPONSE_OP_FAILED                4

#define     GATT_BATTERY_SERVICE_UUID               BLEUUID((uint16_t)0x180F)
#define     GATT_USERDATA_SERVICE_UUID              BLEUUID((uint16_t)0x181C)
#define     GATT_BATTERYLEVEL_UUID                  BLEUUID((uint16_t)0x2A19)
#define     GATT_GENERICLEVEL_UUID                  BLEUUID((uint16_t)0X2AF9)

