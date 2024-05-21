/*  Teensy_sensors.ino
 *  Cornell Cup Robotics
 *  Last modified: May 1, 2021
 *
 *  The all-encompassing script for the peripheral sensors on C1C0 used to
 *  aid path planning and locomotion.
 *  Sensors added - Terabee IR ToF 1, Terabee IR ToF 2, LIDAR
 *
 *   Authors - Brett Sawka, Stefen Pegels, Aparajito Saha
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "RPLidar.h"
#include "R2Protocol/R2Protocol.h"
#include <EEPROM.h>

// Macro to set if print statements for sensor data receiving is desired
#define DEBUG 0

//Serial port macros
#define JETSON_SERIAL_PORT Serial5 
#define LIDAR_SERIAL_PORT Serial3
#define TERABEE_1_SERIAL_PORT Serial1

//macros for finding points in terabee serial message
#define START1 77
#define START2 70

#define MSG_INIT 0
#define MSG_BEGIN 10
#define MSG_DATA 20
#define MSG_MASK 30
#define MSG_CRC8 40
#define MSG_DONE 50

#define RPLIDAR_MOTOR 3 // PWM pin for controlling RPLIDAR motor speed - connect to MOTOCTRL

//Macros for appending sensor data into one array
#define LIDAR_DATA_POINTS 360
#define MAX_BUFFER_SIZE 2048
#define TERABEE_DATA_LEN 16
#define LIDAR_DATA_LEN LIDAR_DATA_POINTS * 4
#define SENSOR_DATA_LEN (TERABEE_DATA_LEN * 3 + LIDAR_DATA_LEN)

// Macros for indexes of specific sensor data in sensor_data_buffer
#define TB1_START 0
#define TB2_START TB1_START + TERABEE_DATA_LEN
#define TB3_START TB2_START + TERABEE_DATA_LEN
#define LIDAR_START TB3_START + TERABEE_DATA_LEN

// Terabee Variables
int test_var = 0;
int state;
uint8_t terabee1_databuffer[16] = {0};
uint16_t terabee1_data[8] = {0};
int state2;
uint8_t terabee2_databuffer[16] = {0};
uint16_t terabee2_data[8] = {0};
int state3;
uint8_t terabee3_databuffer[16] = {0};
uint16_t terabee3_data[8] = {0};

// Lidar variables
uint16_t LidarData[LIDAR_DATA_POINTS * 2]; // replace with fixed length and clear/run again if needed
int array_counter = 0;
const uint16_t zero_b16 = 0;
uint8_t lidar_databuffer[LIDAR_DATA_POINTS * 4];
int lidar_array_index;

// Total sensor data buffer where all other sensor data goes in
uint8_t sensor_databuffer[SENSOR_DATA_LEN] = {0};

// Variables to Read From Jetson
uint8_t *permission;
bool waitingForPermission = 1;
uint8_t read_buffer[17];
uint32_t read_buffer_len = 17;
uint16_t read_checksum;
char read_type[4];
uint8_t read_data[1] = {0};
uint32_t read_data_len = 1;
int count;

RPLidar lidar; // Instantiate lidar

//parses serial port for terabee message
void find_msg(int &state, HardwareSerial &ser)
{
  uint8_t b = ser.read();
  switch (state)
  {
  case MSG_INIT:
    if (b == START1)
    {
      state = MSG_BEGIN;
    }
    break;
  case MSG_BEGIN:
    if (b == START2)
    {
      state = MSG_DATA;
    }
    else
    {
      state = MSG_INIT;
    }
    break;
  default:
    state = MSG_INIT;
  }
}

//converts an array of uint8_t to array of uint16_t
void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data)
{
  int data_idx;
  for (int i = 0; i < 16; i++)
  {
    data_idx = i / 2;
    if ((i & 1) == 0)
    {
      // even
      data[data_idx] = databuffer[i] << 8;
    }
    else
    {
      // odd
      data[data_idx] |= databuffer[i];
    }
  }
}

//converts an array of uint16_t to array of uint8_t
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

byte mode[4] = {0x00, 0x11, 0x02, 0x4C};

inline void reset_input_buffer()
{
  while (Serial4.available() > 0)
    Serial4.read();
  delay(100);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);  // Monitor
  Serial4.begin(115200); // Arbiter Teensy
  Serial7.begin(115200); // Terabee3
  lidar.begin(LIDAR_SERIAL_PORT); // Lidar Initialization
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  delay(500); // take some time

  Serial4.write(mode, 4); // write the command for hex output
                          //  Serial4.write(mode, 4); // write the command for hex output
  Serial7.write(mode, 4);
  reset_input_buffer();

  permission = &(read_data[0]);

Serial.println("Stuff");
}

uint8_t b;
int avail;
int avail2;
int avail3;

uint8_t terabee1_send_buffer[MAX_BUFFER_SIZE];
uint8_t terabee2_send_buffer[MAX_BUFFER_SIZE];
uint8_t terabee3_send_buffer[MAX_BUFFER_SIZE];
uint8_t lidar_send_buffer[MAX_BUFFER_SIZE];
uint8_t sensor_send_buffer[MAX_BUFFER_SIZE];

void send(char type[5], const uint8_t *data, uint32_t data_len, uint8_t *send_buffer)
{
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, MAX_BUFFER_SIZE);
  JETSON_SERIAL_PORT.write(send_buffer, written);
  if(DEBUG)
    Serial.println("NUMBER OF BYTES WRITTEN! READ ME " + String(written));
}
void loop()
{
  // Terabee 1 Code
  avail = TERABEE_1_SERIAL_PORT.available();
  Serial.println(TERABEE_1_SERIAL_PORT.available());
  if (avail > 0)
  {
    if (DEBUG)
      Serial.println("Reading TB1");
    if (state == MSG_INIT || state == MSG_BEGIN)
    {
      find_msg(state, TERABEE_1_SERIAL_PORT);
    }
    else if (state == MSG_DATA)
    {
      TERABEE_1_SERIAL_PORT.readBytes(sensor_databuffer + TB1_START, 16);
      state = MSG_INIT;
      // convert_b8_to_b16(terabee1_databuffer, terabee1_data);
      //       for (int i = 0; i < 8; i++) {
      //         Serial.print("Sensor1 ");
      //         Serial.print(i);
      //         Serial.print(": ");
      //         Serial.println(terabee1_data[i]);
      //       }
    }
  }

  // Lidar Code
   //Serial.println(IS_OK(lidar.waitPoint()));
  if (IS_OK(lidar.waitPoint()))
  {
    uint16_t distance = (uint16_t)lidar.getCurrentPoint().distance; // distance value in mm unit
    //Serial.println(distance);
    uint16_t angle = (uint16_t)lidar.getCurrentPoint().angle;       // angle value in degrees

    //        Serial.println("Angle:" + String(angle));
    //        Serial.println("Distance:" + String(distance));
    if (lidar_array_index <= LIDAR_DATA_POINTS - 1)
    {
      LidarData[lidar_array_index * 2] = angle;
      LidarData[lidar_array_index * 2 + 1] = distance;
      lidar_array_index++;
    }
  }
  else
  {

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100)))
    {
      // detected...
      lidar.startScan();
      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
  if (lidar_array_index == LIDAR_DATA_POINTS)
  {
    convert_b16_to_b8(LidarData, sensor_databuffer + LIDAR_START, LIDAR_DATA_POINTS * 2);
    lidar_array_index = 0;
    if (1)
    {
      // delay(10);
      // Copying data arrays into overall sensor data buffer
      send((char *)"SENS", sensor_databuffer, SENSOR_DATA_LEN, sensor_send_buffer);
    }
  }
}
