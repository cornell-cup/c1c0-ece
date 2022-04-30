#include <LobotServoController.h>

// J1: 1500 - 2500
// J2: 500 - 2500
// J3: 500 - 2000 (2500 on App)
// J4: (max range depends on J5) 500 -2500
// J5: 500 - 2500
// J6: 500 - 2500

/*************** Move Individual Motors *****************/
int J1_deg_to_pos (float deg){
  int lower = 1500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return int(pos);
}

int J2_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J3_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2000; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J4_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J5_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J6_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}
/*************** End Move Individual Motors *****************/

int convert_deg_to_pos (int motor, float deg) {
  switch (motor){
    case 1: J1_deg_to_pos(deg);
    case 2: J2_deg_to_pos(deg);
    case 3: J3_deg_to_pos(deg);
    case 4: J4_deg_to_pos(deg);
    case 5: J5_deg_to_pos(deg);
    case 6: J6_deg_to_pos(deg);
    default: -1; 
  }
}

LobotServoController arm(Serial1);

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial2.begin(9600);
// while(!Serial1);
}

float angle6 = 120.0; 
float angle5 = 120.0;
float angle4 = 120.0; 
float angle3 = 120.0;
float angle2 = 120.0; 
float angle1 = 120.0;

void loop() { 
  //Serial.println(Serial1.readString());   
  
  // Parse the string into angles
  if (Serial2.available() > 0) {
    String pos_str = Serial2.readString();
    int idx_begin = 0;
    int idx_end = pos_str.length();
    int idx_comma = pos_str.indexOf(',', idx_begin);
    angle6 = pos_str.substring(idx_begin, idx_comma).toFloat();
    idx_begin = idx_comma+1;
    idx_comma = pos_str.indexOf(',', idx_begin);
    angle5 = pos_str.substring(idx_begin, idx_comma).toFloat();
    idx_begin = idx_comma+1;
    idx_comma = pos_str.indexOf(',', idx_begin);
    angle4 = pos_str.substring(idx_begin, idx_comma).toFloat();
    idx_begin = idx_comma+1;
    idx_comma = pos_str.indexOf(',', idx_begin);
    angle3 = pos_str.substring(idx_begin, idx_comma).toFloat();
    idx_begin = idx_comma+1;
    angle2 = pos_str.substring(idx_begin, idx_end).toFloat();

    Serial.println("inside loop");
    Serial.println(angle6);
    Serial.println(angle5);
    Serial.println(angle4);
    Serial.println(angle3);
    Serial.println(angle2);
  }
//  Serial.println(angle6);
//  Serial.println(angle5);
//  Serial.println(angle4);
//  Serial.println(angle3);
//  Serial.println(angle2);
 
  //arm.moveServos(5, 1000, 2, J2_deg_to_pos(angle2), 3, J3_deg_to_pos(angle3), 4, J4_deg_to_pos(angle4), 5, J5_deg_to_pos(angle5), 6, J6_deg_to_pos(angle6));
  // arm.moveServos(4, 1000, 1, J1_deg_to_pos(90), 2, 1000, 3, 1000, 4, 1000);
  // delay(1000); 
  // arm.moveServos(4, 1000, 1, J1_deg_to_pos(0), 2, 1000, 3, 1000, 4, 1000);
  // delay(1000);
  //Serial.println(J1_deg_to_pos (90));
  //arm.moveServos(1,2500,2,1000);
  //delay(1000);
  //Serial.println(J1_deg_to_pos (10));
  arm.moveServos(2,500,1,J1_deg_to_pos (90),2,J2_deg_to_pos (90));
  Serial.println(J1_deg_to_pos (90));
  delay(1000);
  arm.moveServos(2,500,1,J1_deg_to_pos (120),2,J2_deg_to_pos (60));
  delay(1000);
  
}
