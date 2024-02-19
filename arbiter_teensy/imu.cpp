#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// IMU variables
/*This IS IMPORTANT, added IMU code*/
#define CALIBRATION 0
sensors_event_t event;
uint8_t imu_databuffer[6];
uint16_t imu_data[3];
uint16_t oldbuf[3];
bool foundCalib;

Adafruit_BNO055 bno = Adafruit_BNO055(55); // Instantiate IMU

void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
}

void imu_begin()
{

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
    Serial.println("BNO5 detected");
    bno.enterNormalMode();

    while (Serial1.available() > 0)
    {
        Serial1.read();
        delay(100);
    }

    /*
    Added IMU code
    */
    Serial.println("Orientation Sensor Test");
    Serial.println("");

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
     *  Look for the sensor's unique ID at the beginning oF EEPROM.
     *  This isn't foolproof, but it's better than nothing.
     */
    bno.getSensor(&sensor);
    Serial.println("got sensor");
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    /* Crystal must be configured AFTER loading calibration data into BNO055. */
    bno.setExtCrystalUse(true);

#if CALIBRATION == 1
    if (foundCalib)
    {
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
#endif
    /*
    End of added IMU code in setup
    */
}

void imu_get_data(uint16_t *buf)
{
    // grab old copy
    oldbuf[0] = buf[0];
    oldbuf[1] = buf[1];
    oldbuf[2] = buf[2];

    // update with new imu values
    bno.getEvent(&event);
    buf[0] = (int16_t)event.orientation.x;
    buf[1] = (int16_t)event.orientation.y;
    buf[2] = (int16_t)event.orientation.z;

    // if zerod, send old copy
    if ((buf[0] == 0) && (buf[1] == 0) && (buf[2] == 0)) {
      buf[0] = oldbuf[0];
      buf[1] = oldbuf[1];
      buf[2] = oldbuf[2];
    }
    Serial.println("IMU Data:");
    for (int i=0; i<3; i++){
      Serial.print(buf[i]);
      Serial.print(" ");
    }
    Serial.println("");
}