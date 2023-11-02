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


// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal   

float arraydata[50][2];

int arrayindex;
unsigned long timeBegin;

uint8_t buffdata[100];
                        
void setup() {
  
  // bind the RPLIDAR driver to the arduino hardware serial
 Serial3.begin(115200);
 Serial.begin(9600);
 Serial4.begin(9600);
 Serial.println("Here");
 lidar.begin(Serial3);
 Serial.println("Here");

 arrayindex=0;
 timeBegin = micros();
}

void loop() {
  Serial.println("Here0");
    if (IS_OK(lidar.waitPoint())) {
        Serial.println("Here1"); 
        float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
        float angle    = lidar.getCurrentPoint().angle; //angle value in degrees 
//        Serial.println("Angle:" + String(angle));
//        Serial.println("Distance:" + String(distance));
        if (arrayindex<=49){
          //Serial.println("Index: " +String(arrayindex));
          arraydata[arrayindex][0]=angle;
          arraydata[arrayindex][1]=distance;
          arrayindex = arrayindex+1;
        }
        if (arrayindex==50){
          unsigned long timeEnd = micros();
          unsigned long totalduration = timeEnd-timeBegin;
          Serial.println("Total Duration: " +String(totalduration));
          arrayindex=100;
        }
    } else {
      Serial.println("Here2");
      // try to detect RPLIDAR... 
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
         // detected...
         lidar.startScan();
         // start motor rotating at max allowed speed
         //analogWrite(RPLIDAR_MOTOR, 255);
         delay(1000);
      }
    }
   Serial.println("Here3");
   makeBuffs(arraydata);
    
   Serial4.write(buffdata, sizeof(buffdata));
   Serial.println(buffdata[99]);
}

void makeBuffs(float data[][2]) {

  Serial.println("Here(makeBuffs)");
  for(int i = 0; i < 50; i++){
    for(int j = 0; j < 2; j++) {

      buffdata[i*2+j]=data[i][j];
    }
  }
}
