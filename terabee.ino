/*  terabee.ino
 *  Cornell Cup Robotics
 *  February 26, 2021
 *  
 *  Read messages from terabee ToF sensors on UART line 1
 *  Print out readings in mm from each sensor
 * 
 */

#define START1 77
#define START2 70

#define MSG_INIT 0
#define MSG_BEGIN 10
#define MSG_DATA 20
#define MSG_MASK 30
#define MSG_CRC8 40
#define MSG_DONE 50

int state;
int buff_idx;
uint8_t databuffer[16];
uint16_t data[8];

void find_msg() {
  
  uint8_t b = Serial1.read();
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
        buff_idx = 0;
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
    if ( (i % 2) == 0) {
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

  delay(500); // take some time

  Serial1.write(mode, 4); // write the command for hex output
}

uint8_t  b;
int avail;

void loop() {
  // put your main code here, to run repeatedly:

  avail = Serial1.available();
  if (avail > 0) {
    //Serial.print("Bytes on buffer: ");
    //Serial.println(avail);
    if (state == MSG_INIT || state == MSG_BEGIN) {
      find_msg();
    } else if (state == MSG_DATA) {

      //while(Serial.available() < 16); // wait until all the data is definitely there
      Serial1.readBytes(databuffer, 16);
 
      memcpy(data, databuffer, 16);
      state = MSG_INIT;
      format_data(databuffer, data);
      for (int i = 0; i < 8; i++) {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(data[i]);
      }
    }
    //Serial.println(b);
    
  }
}
