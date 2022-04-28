// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

#include "R2Protocol.h"

// parameters that r2p decode funcion takes in
uint8_t msg_buffer[17];
uint32_t msg_buffer_len = 17;
uint16_t checksum;
char type[5];
uint8_t msg[1];
uint32_t msg_len;
uint8_t recv_buf[2048];


void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(115200); // TX1/RX1 
  Serial3.begin(115200); //DUE 1
  Serial4.begin(115200); //DUE 2

  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }
}

uint16_t buf_idx = 2; //Start at 2 to prevent seg fault (checking for tail bits)

void loop() {
  if (Serial1.available() > 0) {
    //Read a byte until reach stop sequence
    do{
      recv_buf[buf_idx] = Serial1.read();
      buf_idx++;
    }while(recv_buf[buf_idx-3],recv_buf[buf_idx-2],recv_buf[buf_idx-1]] != [0xd2,0xe2,0xf2]); //How to solve the problem of unknown message length?
    
    msg_buffer_len = buf_idx-2;
    msg_len = msg_buffer_len-16;
    buf_idx = 2;
    
    //Serial1.readBytes(msg_buffer, msg_buffer_len);
    r2p_decode(msg_buffer, msg_buffer_len, &checksum, type, msg, &msg_len);
    Serial.println(String(type));

    //Route ENCODED data to correct host, or print out type if not resolved
    if(!strcmp(type,"DUE1")){
      Serial3.write(msg_buffer, msg_buffer_len);
      Serial.println("Wrote to Due 1");
    }
    else if(!strcmp(type,"DUE2")){
      Serial4.write(msg_buffer, msg_buffer_len);
      Serial.println("Wrote to Due 2");
    }
    else{
      Serial.println("Unrecognized Type: "+String(type));
    }

//    for(int i = 0; i < msg_len; i++) {
//      Serial.print((char) msg[i]);
//    }
//    Serial.println();
  }
  
}
