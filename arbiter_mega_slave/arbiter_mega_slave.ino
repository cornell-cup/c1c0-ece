// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

#include "R2Protocol.h"

#define MAX_BUFFER_SIZE 2048

// parameters that r2p decode funcion takes in
uint8_t packet_in_buffer[28];
uint8_t packet_out_buffer[28];
uint32_t packet_in_len = 28;
uint16_t checksum;
char type[5];
uint8_t msg_in[12];
uint8_t msg_out[12] = {8,7,6,5,4,3,45,76,222,11,12,7};
uint32_t msg_len_in = 12;
uint32_t msg_len_out = 12;

unsigned long counter = 0;

void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(115200); // TX1/RX1 to master Teensy

  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }
}

void send(char type[5], const uint8_t* msg, uint32_t msg_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, msg, msg_len, send_buffer, MAX_BUFFER_SIZE);
  Serial1.write(send_buffer, written);
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
  if((++counter & 2047) == 2047)
    send("PRMR", msg_out, msg_len_out, packet_out_buffer);
    
}
