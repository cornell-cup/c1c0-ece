// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

#include "R2Protocol.h"

// parameters that r2p decode funcion takes in
uint8_t msg_buffer[24];
uint32_t msg_buffer_len = 24;
uint16_t checksum;
char type[5];
uint8_t msg[8];
uint32_t msg_len;


void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(115200); // TX1/RX1 to master Teensy

  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }
}

void loop() {
  if (Serial1.available() > 0) {
    //Message from Teensy, from Jetson
    Serial1.readBytes(msg_buffer, msg_buffer_len);
    r2p_decode(msg_buffer, msg_buffer_len, &checksum, type, msg, &msg_len);

    for(int i = 0; i < msg_len; i++) {
      Serial.print((char) msg[i]);
    }
    Serial.println();

    //Send Response Message Upstream
    
  }
  
}
