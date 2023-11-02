/*
 * RoboPeak RPLIDAR Arduino Example
 * This example shows the easy and common way to fetch data from an RPLIDAR
 * 
 * You may freely add your application code based on this template
 *
 * USAGE:
 * ---------------------------------
 * 1. Download this sketch code to your Arduino board
 * 2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
 * 3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3
 */
 
/* 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 * RoboPeak.com
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>
#include <R2Protocol.h>

// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal   
//floor angle and distance to 16bit ints
//uint16_t arraydata[50][2];
int arrayindex;
unsigned long timeBegin;

uint16_t buffdatatemp[100];
uint8_t buffdata[200];
uint8_t buffdatasend[1024];
                        
void setup() {
  
  // bind the RPLIDAR driver to the arduino hardware serial
 Serial3.begin(115200);
 Serial.begin(4800);
 Serial4.begin(38400);
 Serial.println("Here");
 lidar.begin(Serial3);
 Serial.println("Here");
 Serial.println(sizeof(buffdata));

 arrayindex=0;
 timeBegin = micros();
}

void send(char type[5], const uint8_t* data, uint32_t data_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 1024);
  Serial4.write(send_buffer, written);
  Serial.println("NUMBER OF BYTES WRITTEN! READ ME" + String(written));
}

void loop() {
    if (IS_OK(lidar.waitPoint())) {
        uint16_t distance = (uint16_t) lidar.getCurrentPoint().distance; //distance value in mm unit
        uint16_t angle    = (uint16_t) lidar.getCurrentPoint().angle; //angle value in degrees
               
//        Serial.println("Angle:" + String(angle));
//        Serial.println("Distance:" + String(distance));
        if (arrayindex <= 49) {
          buffdatatemp[arrayindex*2] = angle;
          buffdatatemp[arrayindex*2+1] = distance;
          arrayindex++;
          }
        }
     else {
      
      // try to detect RPLIDAR... 
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
         // detected...
         lidar.startScan();
         // start motor rotating at max allowed speed
         analogWrite(RPLIDAR_MOTOR, 255);
         delay(1000);
      }
    }

     if (arrayindex == 50) {
        printArray(buffdatatemp);
        convert_b16_to_b8(buffdatatemp, buffdata,100);
//        for (int i = 0; i < 200; i++)
//          Serial.println(buffdata[i]);
        send("LDR", buffdata, 200, buffdatasend);
        arrayindex=0; 
     }   
}
void printArray(uint16_t data[100]){
  for (int i = 0; i<100; i++){
        if(i%2 == 0){
          Serial.println("Angle: " +String(data[i]));
        }
        else{
          Serial.println("Distance: "+String(data[i]));
        }
      }
    
}

void convert_b16_to_b8(uint16_t *databuffer, uint8_t *data, int len) {
  for (int i = 0; i < 2*len; i+=2) {
    data[i] = (databuffer[i/2] >> 8) & 255;
    data[i+1] = (databuffer[i/2]) & 255;
  }  
}
