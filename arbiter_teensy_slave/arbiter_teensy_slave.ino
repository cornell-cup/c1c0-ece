// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

#include "R2Protocol.h"

#define MAX_BUFFER_SIZE 2048

// parameters that r2p decode funcion takes in
uint8_t packet_in_buffer[24];
uint8_t packet_out_buffer[1574];
uint32_t packet_in_len = 24;
uint16_t checksum;
char type[5];
uint8_t msg_in[8];
uint8_t msg_out[1558] = {0};
uint32_t msg_len_in = 8;
uint32_t msg_len_out = 1558;

unsigned long counter = 0;

void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(115200); // TX1/RX1 to master Teensy

  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }

  //Populate a few bytes for reference
  for(int i = 0; i < msg_len_out; i++){
    msg_out[i] = i & 255;
  }
}

void send(char type[5], const uint8_t* msg, uint32_t msg_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, msg, msg_len, send_buffer, MAX_BUFFER_SIZE);
  Serial1.write(send_buffer, written);
  Serial1.flush();
  Serial.println("NUMBER OF BYTES WRITTEN: " + String(written));
}

void loop() {
  if (Serial1.available() > 0) { 
    //Message from arbiter
    Serial1.readBytes(packet_in_buffer, packet_in_len);
    r2p_decode(packet_in_buffer, packet_in_len, &checksum, type, msg_in, &msg_len_in);

//    for(int i = 0; i < msg_len; i++) {
//      Serial.print((char) msg[i]);
//    }
//    Serial.println();
  }
  //if((++counter & 255) == 255)
  send("SNSR", msg_out, msg_len_out, packet_out_buffer);
    
}
