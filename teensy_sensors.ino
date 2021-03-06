/*  Teensy_sensors.ino
 *  Cornell Cup Robotics
 *  February 26, 2021
 *  
 *  Read messages from terabee ToF sensors on UART line 1
 *  Print out readings in mm from each sensor
 * 
 */

 /* I2C Info
  *  IMU Device: Adafruit BNO055
  *  Device address -> 
  */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define START1 77
#define START2 70

#define MSG_INIT 0
#define MSG_BEGIN 10
#define MSG_DATA 20
#define MSG_MASK 30
#define MSG_CRC8 40
#define MSG_DONE 50

int state;
uint8_t databuffer[16];
uint16_t data[8];
int state2;
uint8_t databuffer2[16];
uint16_t data2[8];

Adafruit_BNO055 bno = Adafruit_BNO055(55);


void find_msg(int &state, HardwareSerial &ser) {
  
  uint8_t b = ser.read();
  switch(state) {
    case MSG_INIT:
      if (b == START1) {
        state = MSG_BEGIN;
      }
    break;
    case MSG_BEGIN:
      if (b == START2) {
        state = MSG_DATA;
      } else {
        state = MSG_INIT;
      }
    break;
    default:
      state = MSG_INIT;
  }
}


void format_data(uint8_t *databuffer, uint16_t *data) {
  int data_idx;
  //uint16_t tmp;
  for (int i=0; i < 16; i++) {
    data_idx = i / 2;
    if ( (i & 1) == 0) {
      // even
      data[data_idx] = databuffer[i] << 8;
    } else {
      //tmp = databuffer[i];
      data[data_idx] |= databuffer[i];
    }
  }
}

byte mode[4] = {0x00,0x11,0x02,0x4C};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  Serial1.begin(115200);
  Serial2.begin(115200);
  bno.begin(); //IMU Initialization

  delay(500); // take some time

  Serial1.write(mode, 4); // write the command for hex output
  Serial2.write(mode, 4); // write the command for hex output
  bno.setExtCrystalUse(true);
}

uint8_t  b;
int avail;
int avail2;

void loop() {
  // put your main code here, to run repeatedly:
  //IMU Vectors
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//  imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  

  avail = Serial1.available();
  if (avail > 0) {
    //Serial.print("Bytes on buffer: ");
    //Serial.println(avail);
    if (state == MSG_INIT || state == MSG_BEGIN) {
      find_msg(state, Serial1);
    } else if (state == MSG_DATA) {

      //while(Serial.available() < 16); // wait until all the data is definitely there
      Serial1.readBytes(databuffer, 16);

      state = MSG_INIT;
      format_data(databuffer, data);
      for (int i = 0; i < 8; i++) {
        Serial.print("Sensor1 ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(data[i]);
      }
    }
  }
  //Serial.println(b);
  avail2 = Serial2.available();
  if (avail2 > 0) {
    //Serial.print("Bytes on buffer: ");
    //Serial.println(avail);
    if (state2 == MSG_INIT || state2 == MSG_BEGIN) {
      find_msg(state2, Serial2);
    } else if (state2 == MSG_DATA) {

      //while(Serial.available() < 16); // wait until all the data is definitely there
      Serial2.readBytes(databuffer2, 16);
 
      state2 = MSG_INIT;
      format_data(databuffer2, data2);
      for (int i = 0; i < 8; i++) {
        Serial.print("Sensor2 ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(data2[i]);
      }
    }
  }
  Serial.print("X: ");
  Serial.print(gyro.x());
  Serial.print(" Y: ");
  Serial.print(gyro.y());
  Serial.print(" Z: ");
  Serial.print(gyro.z());
  Serial.println("");
}
