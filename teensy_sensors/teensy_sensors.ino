/*  Teensy_sensors.ino
 *  Cornell Cup Robotics
 *  Last modified: May 1, 2021
 *  
 *  The all-encompassing script for the peripheral sensors on C1C0 used to 
 *  aid path planning and locomotion.
 *  Sensors added - Terabee IR ToF 1, Terabee IR ToF 2, LIDAR, Adafruit BNO055
 *   
 *   Authors - Brett Sawka, Stefen Pegels, Aparajito Saha
 */

 /*  I2C Info
  *  IMU Device: Adafruit BNO055
  *  Device address -> 
  */

#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
#include <RPLidar.h>
#include <R2Protocol.h>

#define START1 77
#define START2 70

#define MSG_INIT 0
#define MSG_BEGIN 10
#define MSG_DATA 20
#define MSG_MASK 30
#define MSG_CRC8 40
#define MSG_DONE 50

#define RPLIDAR_MOTOR 3 // PWM pin for controlling RPLIDAR motor speed - connect to MOTOCTRL

//Terabee Variables
int state;
uint8_t terabee1_databuffer[16];
uint16_t terabee1_data[8];
int state2;
uint8_t terabee2_databuffer[16];
uint16_t terabee2_data[8];

//Lidar variables
unsigned int LidarData[100]; //replace with fixed length and clear/run again if needed
int array_counter = 0;
const uint16_t zero_b16 = 0;
uint8_t lidar_databuffer[200];
int lidar_array_index;

//Adafruit_BNO055 bno = Adafruit_BNO055(55); // Instantiate IMU

RPLidar lidar; // Instantiate lidar

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

void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data) {
  int data_idx;
  for (int i=0; i < 16; i++) {
    data_idx = i / 2;
    if ( (i & 1) == 0) {
      // even
      data[data_idx] = databuffer[i] << 8;
    } else {
      // odd
      data[data_idx] |= databuffer[i];
    }
  }
}

void convert_b16_to_b8(int *databuffer, uint8_t *data, int len) {
//  int data_idx1;
//  int data_idx2;
  for (int i = 0; i < 2*len; i+=2) {
    data[i] = (databuffer[i/2] >> 8) & 255;
    data[i+1] = (databuffer[i/2]) & 255;
  }  
}

byte mode[4] = {0x00,0x11,0x02,0x4C};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);   //Monitor 
  Serial1.begin(115200); //Terabee1
  Serial2.begin(115200); //Terabee2
  Serial3.begin(115200); //Lidar
  Serial4.begin(9600);
  Serial5.begin(115200); // Jetson
//  bno.begin();           //IMU Initialization
//  bno.enterNormalMode();
  lidar.begin(Serial3);  //Lidar Initialization

  delay(500); // take some time

  Serial1.write(mode, 4); // write the command for hex output
  Serial2.write(mode, 4); // write the command for hex output
//  bno.setExtCrystalUse(true);
}

uint8_t  b;
int avail;
int avail2;

uint8_t terabee1_send_buffer[1024];
uint8_t terabee2_send_buffer[1024];
uint8_t lidar_send_buffer[1024];

void send(char type[5], const uint8_t* data, uint32_t data_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 1024);
  Serial4.write(send_buffer, written);
  Serial.println("NIMBER OF BYTES WRITTEN! READ ME" + String(written));
}

void loop() {
 
  //Terabee 1 Code
  avail = Serial1.available();
  if (avail > 0) {
    if (state == MSG_INIT || state == MSG_BEGIN) {
      find_msg(state, Serial1);
    } else if (state == MSG_DATA) {
      Serial1.readBytes(terabee1_databuffer, 16);
      state = MSG_INIT;
      send("IR", terabee1_databuffer, 16, terabee1_send_buffer);
      convert_b8_to_b16(terabee1_databuffer, terabee1_data);
      for (int i = 0; i < 8; i++) {
        Serial.print("Sensor1 ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(terabee1_data[i]);
      }
      //IMU Vectors
//        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//        imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      //IMU Code
//      Serial.print("X: ");
//      Serial.print(gyro.x());
//      Serial.print(" Y: ");
//      Serial.print(gyro.y());
//      Serial.print(" Z: ");
//      Serial.print(gyro.z());
//      Serial.println("");
//  
//      Serial.println("LIDAR array count = ");
//      Serial.println(array_counter);
//      convert_b16_to_b8(LidarData, lidar_databuffer, 360);
//      send("LDR", lidar_databuffer, 720, lidar_send_buffer);
//      array_counter = 0;
//      for (int i = 0; i < 360; i++) {
//        LidarData[i] = zero_b16;
//      }
    }
  }
//
////  // Terabee 2 code
//  
//  avail2 = Serial2.available();
//  if (avail2 > 0) {
//    if (state2 == MSG_INIT || state2 == MSG_BEGIN) {
//      find_msg(state2, Serial2);
//    } else if (state2 == MSG_DATA) {
//      Serial2.readBytes(terabee2_databuffer, 16);
//      state2 = MSG_INIT;
//      convert_b8_to_b16(terabee2_databuffer, terabee2_data);
//      send("IR2", terabee2_databuffer, 16, terabee2_send_buffer);
//      for (int i = 0; i < 8; i++) {
//        Serial.print("Sensor2 ");
//        Serial.print(i);
//        Serial.print(": ");
//        Serial.println(terabee2_data[i]);
//      }
//    }
//  }
//  
//  
//  //Lidar Code
//  if (IS_OK(lidar.waitPoint())) {
//        uint16_t distance = (uint16_t) lidar.getCurrentPoint().distance; //distance value in mm unit
//        uint16_t angle    = (uint16_t) lidar.getCurrentPoint().angle; //angle value in degrees
//               
//        //Serial.println("Angle:" + String(angle));
//        //Serial.println("Distance:" + String(distance));
//        if (lidar_array_index <= 49) {
//          LidarData[lidar_array_index*2] = angle;
//          LidarData[lidar_array_index*2+1] = distance;
//          lidar_array_index++;
//          }
//        }
//     else {
//      
//      // try to detect RPLIDAR... 
//      rplidar_response_device_info_t info;
//      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
//         // detected...
//         lidar.startScan();
//         // start motor rotating at max allowed speed
//         analogWrite(RPLIDAR_MOTOR, 255);
//         delay(1000);
//      }
//    }
//
//     if (lidar_array_index == 50) {
////        printArray(buffdatatemp);
//        convert_b16_to_b8(LidarData, lidar_databuffer,100);
////        for (int i = 0; i < 200; i++)
////          Serial.println(buffdata[i]);
//        send("LDR", lidar_databuffer, 200, lidar_send_buffer);
//        lidar_array_index=0; 
//     }   
}
