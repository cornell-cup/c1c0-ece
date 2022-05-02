// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

#include "R2Protocol.h"

// parameters that r2p decode funcion takes in
uint8_t msg_buffer[19];
uint32_t msg_buffer_len = 19;
uint16_t checksum;
char type[5];
uint8_t msg[3];
uint32_t msg_len;
uint8_t recv_buf[2048];
uint8_t iter_counter = 0;

uint16_t buf_idx = 0;

uint8_t prev2=0, prev1=0, prev0=0;
uint8_t *p_prev2=&prev2, *p_prev1=&prev1, *p_prev0=&prev0;

uint8_t end_seq[3] = {0xd2,0xe2,0xf2};
uint8_t * end_arr[3] = {p_prev2, p_prev1, p_prev0};

uint8_t recv_buf2[2048], recv_buf3[2048], recv_buf4[2048], recv_buf5[2048], recv_buf6[2048];

//Need to declare these for each connected device
uint16_t msg_len3 = 24; //8 data bytes
char type3[] = "PRM";
char type3_t[] = "PRMT";

uint16_t msg_len4 = 24; //8 data bytes
char type4[] = "LOC";
char type4_t[] = "LOCT";
  
void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(115200); // TX1/RX1 

  //These need to be declared in order to read and send messages to the respective devices
  Serial3.begin(115200); //DUE 1

  Serial4.begin(115200); //DUE 2


  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }
}



inline uint8_t end_cmp(uint8_t * end_arr[3]){
  return ((*end_arr[0]) == end_seq[0]) && ((*end_arr[1]) == end_seq[1]) && ((*end_arr[2]) == end_seq[2]);
}

void loop() {
//  for(int i = 0; i< 3; i++) Serial.print(String((*end_arr[i])) + " ");
//  Serial.println();
//  prev0 = 0xf2;
//  prev1 = 0xe2;
//  prev2 = 0xd2;
//  delay(1000);
//  Serial.println("Compared: " + String(end_cmp(end_arr)));

  if (Serial1.available() > 17) { //At least one message has arrived, smallest message is 17 bytes (with new R2P)
    //Serial.println(Serial1.available());
    //Read a byte until reach stop sequence
    do{
      recv_buf[buf_idx] = Serial1.read();
      *p_prev2 = *p_prev1;
      *p_prev1 = *p_prev0;
      *p_prev0 = recv_buf[buf_idx];
      buf_idx++;
    }while(Serial1.available() && !end_cmp(end_arr)); //While there are more bytes and we haven't reached the stop sequence

    if(end_cmp(end_arr)) {
      msg_buffer_len = buf_idx;
      msg_len = msg_buffer_len-16;//17 with new r2p
      //Serial.println(msg_len);
      buf_idx = 0;
      
      //Serial1.readBytes(recv_buf, msg_buffer_len);
      r2p_decode(recv_buf, msg_buffer_len, &checksum, type, msg, &msg_len);
      //Route ENCODED data to correct host, or print out type if not resolved
      
      if(!strcmp(type,type3)){ //Downstream Serial3
        Serial3.write(recv_buf, msg_buffer_len);
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type3_t)){ //Upstream Serial3
        Serial3.readBytes(recv_buf3, msg_len3);
        Serial.println("Received "+String(type)+" data");
      }
      else if(!strcmp(type,type4)){ //Downstream Serial4
        Serial4.write(recv_buf, msg_buffer_len);
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type4_t)){ //Upstream Serial4
        Serial4.readBytes(recv_buf4, msg_len4);
        Serial.println("Received "+String(type)+" data");
      }
      else{
        Serial.println("Unrecognized Type: "+String(type));
      }
    }
    

//    for(int i = 0; i < msg_len; i++) {
//      Serial.print((char) msg[i]);
//    }
//    Serial.println();
  }
  
}
