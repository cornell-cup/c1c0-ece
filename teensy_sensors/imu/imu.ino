#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
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

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55); // Instantiate IMU

byte mode[4] = {0x00,0x11,0x02,0x4C};

void convert_b16_to_b8(int *databuffer, uint8_t *data, int len) {
//  int data_idx1;
//  int data_idx2;
  for (int i = 0; i < 2*len; i+=2) {
    data[i] = (databuffer[i/2] >> 8) & 255;
    data[i+1] = (databuffer[i/2]) & 255;
  }  
}

void send(char type[5], const uint8_t* data, uint32_t data_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 28);
  Serial4.write(send_buffer, written);
  Serial.println("NIMBER OF BYTES WRITTEN! READ ME" + String(written));
}

int imudata[6];
uint8_t buffdata[12];
uint8_t buffdatasend[28];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);   //Monitor 
  Serial1.begin(115200); //Terabee1
  Serial2.begin(115200); //Terabee2
  Serial3.begin(115200); //Lidar
  Serial4.begin(38400);  //Jetson
  bno.begin();           //IMU Initialization
  bno.enterNormalMode();

  delay(500); // take some time

  Serial1.write(mode, 4); // write the command for hex output
  Serial2.write(mode, 4); // write the command for hex output
  bno.setExtCrystalUse(true);

}

void loop() {
  
      //IMU Vectors
        //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        //imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      //IMU Code
      Serial.print("X: ");
      Serial.print(gyro.x());
      Serial.print(" Y: ");
      Serial.print(gyro.y());
      Serial.print(" Z: ");
      Serial.print(gyro.z());
      Serial.println("");
      Serial.println("Linear Accel");
      Serial.print("X: ");
      Serial.print(lin_accel.x());
      Serial.print(" Y: ");
      Serial.print(lin_accel.y());
      Serial.print(" Z: ");
      Serial.print(lin_accel.z());
      Serial.println("");

      imudata[0] = (int)gyro.x();
      imudata[1] = (int)gyro.y();
      imudata[2] = (int)gyro.z();
      imudata[3] = (int)lin_accel.x();
      imudata[4] = (int)lin_accel.y();
      imudata[5] = (int)lin_accel.z();

      convert_b16_to_b8(imudata,buffdata,6);
      send("IMU", buffdata, 12, buffdatasend);

      delay(BNO055_SAMPLERATE_DELAY_MS);
  

}
