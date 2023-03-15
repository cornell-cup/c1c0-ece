// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit
// Author: Stefen Pegels, email sgp62@cornell.edu for contact

/*  I2C Info
 *  IMU Device: Adafruit BNO055
 *  Device address ->
 */

#include "R2Protocol.h"
/* ADDED IMU SETTING*/
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

//Uncomment which serial ports are in use, make sure to define their parameters below
//#define SER2
#define SER3
//#define SER4
//#define SER5
//#define SER6
//#define SER7

//imu define
#define BNO055_SAMPLERATE_DELAY_MS (100)

//TEMPORARILY HERE, TRYING TO CLEAN UP SO NO OTHER DATA INVOLVED
#define MAX_BUFFER_SIZE 2048

//THIS IS IMPORTANT
#define IMU_DATA_LEN 6

//ORIGINALLY FOR SENSOR DATA BUFFER, DON'T THINK WE SHOULD NEED IT.
#define IMU_START TERABEE_DATA_LEN * 3 + LIDAR_DATA_LEN


//Need to declare these for each connected device

#ifdef SER2
uint8_t STATE2 = 0;
uint8_t recv_buf2[2048]; //Buffer for upstream messages coming from Ser2 device, to be forwarded to jetson upon request
uint16_t msg_len2 = 28; //upstream packet length (data len) + 16
char type2_d[] = "PRM"; //Downstream type
char type2_u[] = "PRMR"; //Upstream type
uint16_t ser2_count = 0;
#endif

#ifdef SER3
uint8_t STATE3 = 0;
uint8_t recv_buf3[2048];
uint16_t msg_len3 = 1574; //upstream packet length (data len) + 16
char type3_d[] = "SNS"; //Downstream type
char type3_u[] = "SNSR"; //Upstream type
uint16_t ser3_count = 0;
#endif

#ifdef SER4
uint8_t STATE4 = 0;
uint8_t recv_buf4[2048];
uint16_t msg_len4 = 29; //upstream packet length (data len) + 16
char type4_d[] = "loco"; //Downstream type
char type4_u[] = "LOCR"; //Upstream type (NOT NEEDED FOR PATH_PLANNING)
uint16_t ser4_count = 0;
#endif

#ifdef SER5
uint8_t STATE5 = 0;
uint8_t recv_buf5[2048];
uint16_t msg_len5 = 0; //upstream packet length (data len) + 16
char type5_d[] = ""; //Downstream type
char type5_u[] = ""; //Upstream type
uint16_t ser5_count = 0;
#endif

#ifdef SER6
uint8_t STATE6 = 0;
uint8_t recv_buf6[2048];
uint16_t msg_len6 = 0; //upstream packet length (data len) + 16
char type6_d[] = ""; //Downstream type
char type6_u[] = ""; //Upstream type
uint16_t ser6_count = 0;
#endif

#ifdef SER7
uint8_t STATE7 = 0;
uint8_t recv_buf7[2048];
uint16_t msg_len7 = 0; //upstream packet length (data len) + 16
char type7_d[] = ""; //Downstream type
char type7_u[] = ""; //Upstream type
uint16_t ser7_count = 0;
#endif



// parameters for r2p decode from jetson downstream
uint32_t msg_buffer_len = 0;
uint16_t checksum;
char type[5];
uint8_t msg[2048];
uint32_t msg_len;
uint8_t recv_buf[2048];

uint16_t buf_idx = 0;

uint8_t prev2=0, prev1=0, prev0=0;
uint8_t *p_prev2=&prev2, *p_prev1=&prev1, *p_prev0=&prev0;

uint8_t end_seq[3] = {0xd2,0xe2,0xf2};
uint8_t * end_arr[3] = {p_prev2, p_prev1, p_prev0};

// IMU variables
/*This IS IMPORTANT, added IMU code*/
sensors_event_t event;
uint8_t imu_databuffer[6];
uint16_t imu_data[3];
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

void convert_b16_to_b8(uint16_t *databuffer, uint8_t *data, int len)
{
  //  int data_idx1;
  //  int data_idx2;
  for (int i = 0; i < 2 * len; i += 2)
  {
    data[i] = (databuffer[i / 2] >> 8) & 255;
    data[i + 1] = (databuffer[i / 2]) & 255;
  }
}
/*
end of added IMU code uphere
*/

void setup() {
  
  Serial.begin(115200); // Serial monitor
  Serial1.begin(115200); // TX1/RX1 

  //These need to be declared in order to read and send messages to the respective devices
  #ifdef SER2
  Serial2.begin(115200);
  #endif
  #ifdef SER3
  Serial3.begin(115200);
  #endif
  #ifdef SER4
  Serial4.begin(115200);
  #endif
  #ifdef SER5
  Serial5.begin(115200);
  #endif
  #ifdef SER6
  Serial6.begin(115200);
  #endif
  #ifdef SER7
  Serial7.begin(115200);
  #endif

  bno.begin(); // IMU Initialization
  bno.enterNormalMode();

  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }

  /*
  Added IMU code
  */
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  //  if (!bno.begin())
  //  {
  //    /* There was a problem detecting the BNO055 ... check your connections */
  //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //    while (1)
  //      ;
  //  }

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

  /* always recal the mag as It goes out of calibration very often */
  //  if (foundCalib)
  //  {
  //    Serial.println("Move sensor slightly to calibrate magnetometers");
  //    while (!bno.isFullyCalibrated())
  //    {
  //      bno.getEvent(&event);
  //      delay(BNO055_SAMPLERATE_DELAY_MS);
  //    }
  //  }
  //  else
  //  {
  //    Serial.println("Please Calibrate Sensor: ");
  //    while (!bno.isFullyCalibrated())
  //    {
  //      bno.getEvent(&event);
  //
  //      Serial.print("X: ");
  //      Serial.print(event.orientation.x, 4);
  //      Serial.print("\tY: ");
  //      Serial.print(event.orientation.y, 4);
  //      Serial.print("\tZ: ");
  //      Serial.print(event.orientation.z, 4);
  //
  //      /* Optional: Display calibration status */
  //      displayCalStatus();
  //
  //      /* New line for the next sample */
  //      Serial.println("");
  //
  //      /* Wait the specified delay before requesting new data */
  //      delay(BNO055_SAMPLERATE_DELAY_MS);
  //    }
  //  }
  //
  //  Serial.println("\nFully calibrated!");
  //  Serial.println("--------------------------------");
  //  Serial.println("Calibration Results: ");
  //  adafruit_bno055_offsets_t newCalib;
  //  bno.getSensorOffsets(newCalib);
  //  displaySensorOffsets(newCalib);
  //
  //  Serial.println("\n\nStoring calibration data to EEPROM...");
  //
  //  eeAddress = 0;
  //  bno.getSensor(&sensor);
  //  bnoID = sensor.sensor_id;
  //
  //  EEPROM.put(eeAddress, bnoID);
  //
  //  eeAddress += sizeof(long);
  //  EEPROM.put(eeAddress, newCalib);
  //  Serial.println("Data stored to EEPROM.");
  //
  //  Serial.println("\n--------------------------------\n");
  //  delay(500);
  /*
  End of added IMU code in setup
  */
}


inline uint8_t end_cmp(uint8_t * end_arr[3]){
  return ((*end_arr[0]) == end_seq[0]) && ((*end_arr[1]) == end_seq[1]) && ((*end_arr[2]) == end_seq[2]);
}

#ifdef SER2
void serialEvent2(){
  if(Serial2.available()){
    recv_buf2[ser2_count++] = Serial2.read();
    if(ser2_count < msg_len2){
      if(STATE2 == 0) {
        if(recv_buf2[0] == 0xa2)
          STATE2 = 0xa2;
        else
          ser2_count = 0;
      }
      else if(STATE2 == 0xa2){
        if(recv_buf2[1] == 0xb2)
          STATE2 = 0xb2;
        else {
          ser2_count = 0;
          STATE2 = 0;
        }
      }
      else if(STATE2 == 0xb2){
        if(recv_buf2[2] == 0xc2)
          STATE2 = 0xc2;
        else {
          ser2_count = 0;
          STATE2 = 0;
        }
      }
    }
  }
  if(ser2_count == msg_len2) {
//    Serial.println(ser2_count);
//    Serial.println(recv_buf2[1573]);
    ser2_count = 0;
    STATE2 = 0;
  }
}
#endif
uint32_t msg_buffer_len3 = 0;
uint16_t checksum3;
char type3[5];
uint8_t msg3[2048];
//CONFLICT DEFINITION WITH uint16_t msg_len3 = 1574; //upstream packet length (data len) + 16, COMMENT OUT RIGHT NOW
//uint32_t msg_len3;
//CONFLICT DEFINITION WITH THE SAME THING ABOVE, COMMENT OUT FOR NOW
//uint8_t recv_buf3[2048];
//Sensor serial port
#ifdef SER3
void serialEvent3(){
  if(Serial3.available()){
//    Serial.println("Reading from ser3");
    recv_buf3[ser3_count++] = Serial3.read();
    if(ser3_count < msg_len3){
      if(STATE3 == 0) {
        if(recv_buf3[0] == 0xa2)
          STATE3 = 0xa2;
        else
          ser3_count = 0;
      }
      else if(STATE3 == 0xa2){
        if(recv_buf3[1] == 0xb2)
          STATE3 = 0xb2;
        else {
          ser3_count = 0;
          STATE3 = 0;
        }
      }
      else if(STATE3 == 0xb2){
        if(recv_buf3[2] == 0xc2)
          STATE3 = 0xc2;
        else {
          ser3_count = 0;
          STATE3 = 0;
        }
      }
    }
  }
  if(ser3_count == msg_len3) {
//    Serial.println(ser3_count);
//    Serial.println(recv_buf3[1573]);
    //r2p_decode(recv_buf3, msg_len3, &checksum3, type3, msg3, &msg_len3);
    ser3_count = 0;

    STATE3 = 0;
  }
}
#endif

#ifdef SER4
void serialEvent4(){
  if(Serial4.available()){
    recv_buf4[ser4_count++] = Serial4.read();
    if(ser4_count < msg_len4){
      if(STATE4 == 0) {
        if(recv_buf4[0] == 0xa2)
          STATE4 = 0xa2;
        else
          ser4_count = 0;
      }
      else if(STATE4 == 0xa2){
        if(recv_buf4[1] == 0xb2)
          STATE4 = 0xb2;
        else {
          ser4_count = 0;
          STATE4 = 0;
        }
      }
      else if(STATE4 == 0xb2){
        if(recv_buf4[2] == 0xc2)
          STATE4 = 0xc2;
        else {
          ser4_count = 0;
          STATE4 = 0;
        }
      }
    }
  }
  if(ser4_count == msg_len4) {
    ser4_count = 0;
    STATE4 = 0;
  }
}
#endif

#ifdef SER5
void serialEvent5(){
  if(Serial5.available()){
    recv_buf5[ser5_count++] = Serial5.read();
    if(ser5_count < msg_len5){
      if(STATE5 == 0) {
        if(recv_buf5[0] == 0xa2)
          STATE5 = 0xa2;
        else
          ser5_count = 0;
      }
      else if(STATE5 == 0xa2){
        if(recv_buf5[1] == 0xb2)
          STATE5 = 0xb2;
        else {
          ser5_count = 0;
          STATE5 = 0;
        }
      }
      else if(STATE5 == 0xb2){
        if(recv_buf5[2] == 0xc2)
          STATE5 = 0xc2;
        else {
          ser5_count = 0;
          STATE5 = 0;
        }
      }
    }
  }
  if(ser5_count == msg_len5) {
    ser5_count = 0;
    STATE5 = 0;
  }
}
#endif

#ifdef SER6
void serialEvent6(){
  if(Serial6.available()){
    recv_buf6[ser6_count++] = Serial6.read();
    if(ser6_count < msg_len6){
      if(STATE6 == 0) {
        if(recv_buf6[0] == 0xa2)
          STATE6 = 0xa2;
        else
          ser6_count = 0;
      }
      else if(STATE6 == 0xa2){
        if(recv_buf6[1] == 0xb2)
          STATE6 = 0xb2;
        else {
          ser6_count = 0;
          STATE6 = 0;
        }
      }
      else if(STATE6 == 0xb2){
        if(recv_buf6[2] == 0xc2)
          STATE6 = 0xc2;
        else {
          ser6_count = 0;
          STATE6 = 0;
        }
      }
    }
  }
  if(ser6_count == msg_len6) {
    ser6_count = 0;
    STATE6 = 0;
  }
}
#endif

#ifdef SER7
void serialEvent7(){
  if(Serial7.available()){
    recv_buf7[ser7_count++] = Serial7.read();
    if(ser7_count < msg_len7){
      if(STATE7 == 0) {
        if(recv_buf7[0] == 0xa2)
          STATE7 = 0xa2;
        else
          ser7_count = 0;
      }
      else if(STATE7 == 0xa2){
        if(recv_buf7[1] == 0xb2)
          STATE7 = 0xb2;
        else {
          ser7_count = 0;
          STATE7 = 0;
        }
      }
      else if(STATE7 == 0xb2){
        if(recv_buf7[2] == 0xc2)
          STATE7 = 0xc2;
        else {
          ser7_count = 0;
          STATE7 = 0;
        }
      }
    }
  }
  if(ser7_count == msg_len7) {
    ser7_count = 0;
    STATE7 = 0;
  }
}
#endif
//Receives variable length messages from Jetson, end_cmp() detects if we have a valid packet
void serialEvent1(){
  if(Serial1.available()){
    recv_buf[buf_idx] = Serial1.read();
    *p_prev2 = *p_prev1;
    *p_prev1 = *p_prev0;
    *p_prev0 = recv_buf[buf_idx];
    buf_idx++;
  }

}

/*
Added IMU_send buffer
*/
uint8_t imu_send_buffer[MAX_BUFFER_SIZE];

/*
Add send function, need it for sending IMU data
*/
void send(char type[5], const uint8_t *data, uint32_t data_len, uint8_t *send_buffer)
{
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, MAX_BUFFER_SIZE);
  Serial4.write(send_buffer, written);
  //Serial.println("NUMBER OF BYTES WRITTEN! READ ME " + String(written));
}

void loop() {
  //Successfully found a packet from the jetson
  if(end_cmp(end_arr)) {
    msg_buffer_len = buf_idx;
    msg_len = msg_buffer_len-16;//17 with new r2p
    buf_idx = 0;
    //Reset end pointers
    *p_prev2 = 0;
    *p_prev1 = 0;
    *p_prev0 = 0;

    r2p_decode(recv_buf, msg_buffer_len, &checksum, type, msg, &msg_len);
    //Route ENCODED data to correct host, or print out type if not resolved
    #ifdef SER2
    if(!strcmp(type,type2_d)){ //Downstream Serial2
      Serial2.write(recv_buf, msg_buffer_len);
      Serial2.flush();
      Serial.println("Wrote "+String(type)+" message");
    }
    else if(!strcmp(type,type2_u)){ //Upstream Serial2
      Serial1.write(recv_buf2, msg_len2);
      Serial1.flush();
      Serial.println("Wrote "+String(type)+" data upstream");
    }
    #endif
    #ifdef SER3
    if(!strcmp(type,type3_d)){ //Downstream Serial3
      Serial3.write(recv_buf, msg_buffer_len);
      Serial3.flush();
      Serial.println("Wrote "+String(type)+" message");
    }
    else if(!strcmp(type,type3_u)){ //Upstream Serial3
      Serial1.write(recv_buf3, msg_len3);
      Serial1.flush();
      Serial.println("Wrote "+String(type)+" data upstream");
    }
    #endif
    #ifdef SER4
    if(!strcmp(type,type4_d)){ //Downstream Serial4
      Serial4.write(recv_buf, msg_buffer_len);
      Serial4.flush();
      Serial.println("Wrote "+String(type)+" message");
    }
    else if(!strcmp(type,type4_u)){ //Upstream Serial4
      Serial1.write(recv_buf4, msg_len4);
      Serial1.flush();
      Serial.println("Wrote "+String(type)+" data upstream");
    }
    #endif
    #ifdef SER5
    if(!strcmp(type,type5_d)){ //Downstream Serial5
      Serial5.write(recv_buf, msg_buffer_len);
      Serial5.flush();
      Serial.println("Wrote "+String(type)+" message");
    }
    else if(!strcmp(type,type5_u)){ //Upstream Serial5
      Serial1.write(recv_buf5, msg_len5);
      Serial1.flush();
      Serial.println("Wrote "+String(type)+" data upstream");
    }
    #endif
    #ifdef SER6
    if(!strcmp(type,type6_d)){ //Downstream Serial6
      Serial6.write(recv_buf, msg_buffer_len);
      Serial6.flush();
      Serial.println("Wrote "+String(type)+" message");
    }
    else if(!strcmp(type,type6_u)){ //Upstream Serial6
      Serial1.write(recv_buf6, msg_len6);
      Serial1.flush();
      Serial.println("Wrote "+String(type)+" data upstream");
    }
    #endif
    #ifdef SER7
    if(!strcmp(type,type7_d)){ //Downstream Serial7
      Serial7.write(recv_buf, msg_buffer_len);
      Serial7.flush();
      Serial.println("Wrote "+String(type)+" message");
    }
    else if(!strcmp(type,type7_u)){ //Upstream Serial7
      Serial1.write(recv_buf7, msg_len7);
      Serial1.flush();
      Serial.println("Wrote "+String(type)+" data upstream");
    }
    #endif
    /*
    Below all added code for IMU
    */
    Serial.println("Reading imu");
    bno.getEvent(&event);
    imu_data[0] = (int)event.orientation.x;
    imu_data[1] = (int)event.orientation.y;
    imu_data[2] = (int)event.orientation.z;

    //convert_b16_to_b8(imu_data, imu_databuffer+IMU_START, 3);
    convert_b16_to_b8(imu_data, imu_databuffer, 3);
    if (1){
      send("IMU", imu_databuffer, 6, imu_send_buffer);
    }
//      else{
//        Serial.println("Unrecognized Type: "+String(type));
//      }
  }
}
