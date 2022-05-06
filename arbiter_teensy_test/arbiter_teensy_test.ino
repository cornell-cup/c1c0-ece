// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

#include "R2Protocol.h"
//Uncomment which serial ports are in use, make sure to define their parameters below
#define SER2
#define SER3
#define SER4
//#define SER5
//#define SER6
//#define SER7

//Need to declare these for each connected device

#ifdef SER2
uint8_t recv_buf2[2048]; //Buffer for upstream messages coming from Ser2 device, to be forwarded to jetson upon request
uint16_t msg_len2 = 28; //upstream packet length (data len) + 16
char type2_d[] = "PRM"; //Downstream type
char type2_u[] = "PRMR"; //Upstream type
#endif

#ifdef SER3
uint8_t recv_buf3[2048];
uint16_t msg_len3 = 1574; //upstream packet length (data len) + 16
char type3_d[] = "SNS"; //Downstream type
char type3_u[] = "SNSR"; //Upstream type
#endif

#ifdef SER4
uint8_t recv_buf4[2048];
uint16_t msg_len4 = 22; //upstream packet length (data len) + 16
char type4_d[] = "LOC"; //Downstream type
char type4_u[] = "LOCR"; //Upstream type
#endif

#ifdef SER5
uint8_t recv_buf5[2048];
uint16_t msg_len5 = 0; //upstream packet length (data len) + 16
char type5_d[] = ""; //Downstream type
char type5_u[] = ""; //Upstream type
#endif

#ifdef SER6
uint8_t recv_buf6[2048];
uint16_t msg_len6 = 0; //upstream packet length (data len) + 16
char type6_d[] = ""; //Downstream type
char type6_u[] = ""; //Upstream type
#endif

#ifdef SER7
uint8_t recv_buf7[2048];
uint16_t msg_len7 = 0; //upstream packet length (data len) + 16
char type7_d[] = ""; //Downstream type
char type7_u[] = ""; //Upstream type
#endif


// parameters for r2p decode from jetson downstream
uint32_t msg_buffer_len = 0;
uint16_t checksum;
char type[5];
uint8_t msg[2048];
uint32_t msg_len;
uint8_t recv_buf[2048];

uint16_t buf_idx = 0;

uint8_t prev2=0, prev1=0, prev0=0;
uint8_t *p_prev2=&prev2, *p_prev1=&prev1, *p_prev0=&prev0;

uint8_t end_seq[3] = {0xd2,0xe2,0xf2};
uint8_t * end_arr[3] = {p_prev2, p_prev1, p_prev0};
  
void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(115200); // TX1/RX1 

  //These need to be declared in order to read and send messages to the respective devices
  #ifdef SER2
  Serial2.begin(115200);
  #endif
  #ifdef SER3
  Serial3.begin(115200);
  #endif
  #ifdef SER4
  Serial4.begin(115200);
  #endif
  #ifdef SER5
  Serial5.begin(115200);
  #endif
  #ifdef SER6
  Serial6.begin(115200);
  #endif
  #ifdef SER7
  Serial7.begin(115200);
  #endif

  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }
}


inline uint8_t end_cmp(uint8_t * end_arr[3]){
  return ((*end_arr[0]) == end_seq[0]) && ((*end_arr[1]) == end_seq[1]) && ((*end_arr[2]) == end_seq[2]);
}

void loop() {
  //Serial.println(Serial3.available());
  //Update input buffers from each connected Serial Device, overwriting the same indices
  #ifdef SER2
  if(Serial2.available() >= msg_len2){ //Continuously update Buffers for connected devices
    Serial2.readBytes(recv_buf2, msg_len2);
  }
  #endif
  #ifdef SER3
  if(Serial3.available() >= msg_len3){ //Continuously update Buffers for connected devices
    Serial3.readBytes(recv_buf3, msg_len3);
    Serial.println("Filled sensor data");
  }
  #endif
  #ifdef SER4
  if(Serial4.available() >= msg_len4){ //Continuously update Buffers for connected devices
    Serial4.readBytes(recv_buf4, msg_len4);
  }
  #endif
  #ifdef SER5
  if(Serial5.available() >= msg_len5){ //Continuously update Buffers for connected devices
    Serial5.readBytes(recv_buf5, msg_len5);
  }
  #endif
  #ifdef SER6
  if(Serial6.available() >= msg_len6){ //Continuously update Buffers for connected devices
    Serial6.readBytes(recv_buf6, msg_len6);
  }
  #endif
  #ifdef SER7
  if(Serial7.available() >= msg_len7){ //Continuously update Buffers for connected devices
    Serial7.readBytes(recv_buf7, msg_len7);
  }
  #endif

  //Receives Messages from Jetson, of variable lengths
  if (Serial1.available() >= 17) { //At least one message has arrived, smallest message is 17 bytes (with new R2P)
    //Read a byte until reach stop sequence
    do{
      recv_buf[buf_idx] = Serial1.read();
      *p_prev2 = *p_prev1;
      *p_prev1 = *p_prev0;
      *p_prev0 = recv_buf[buf_idx];
      buf_idx++;
    }while(Serial1.available() && !end_cmp(end_arr)); //While there are more bytes and we haven't reached the stop sequence

    
    //Successfully found a packet from the jetson
    if(end_cmp(end_arr)) {
      msg_buffer_len = buf_idx;
      msg_len = msg_buffer_len-16;//17 with new r2p
      buf_idx = 0;

      r2p_decode(recv_buf, msg_buffer_len, &checksum, type, msg, &msg_len);
      //Route ENCODED data to correct host, or print out type if not resolved

      #ifdef SER2
      if(!strcmp(type,type2_d)){ //Downstream Serial2
        Serial2.write(recv_buf, msg_buffer_len);
        Serial2.flush();
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type2_u)){ //Upstream Serial2
        Serial1.write(recv_buf2, msg_len2);
        Serial1.flush();
        Serial.println("Wrote "+String(type)+" data upstream");
      }
      #endif
      #ifdef SER3
      if(!strcmp(type,type3_d)){ //Downstream Serial3
        Serial3.write(recv_buf, msg_buffer_len);
        Serial3.flush();
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type3_u)){ //Upstream Serial3
        Serial1.write(recv_buf3, msg_len3);
        Serial1.flush();
        Serial.println("Wrote "+String(type)+" data upstream");
      }
      #endif
      #ifdef SER4
      if(!strcmp(type,type4_d)){ //Downstream Serial4
        Serial4.write(recv_buf, msg_buffer_len);
        Serial4.flush();
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type4_u)){ //Upstream Serial4
        Serial1.write(recv_buf4, msg_len4);
        Serial1.flush();
        Serial.println("Wrote "+String(type)+" data upstream");
      }
      #endif
      #ifdef SER5
      if(!strcmp(type,type5_d)){ //Downstream Serial5
        Serial5.write(recv_buf, msg_buffer_len);
        Serial5.flush();
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type5_u)){ //Upstream Serial5
        Serial1.write(recv_buf5, msg_len5);
        Serial1.flush();
        Serial.println("Wrote "+String(type)+" data upstream");
      }
      #endif
      #ifdef SER6
      if(!strcmp(type,type6_d)){ //Downstream Serial6
        Serial6.write(recv_buf, msg_buffer_len);
        Serial6.flush();
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type6_u)){ //Upstream Serial6
        Serial1.write(recv_buf6, msg_len6);
        Serial1.flush();
        Serial.println("Wrote "+String(type)+" data upstream");
      }
      #endif
      #ifdef SER7
      if(!strcmp(type,type7_d)){ //Downstream Serial7
        Serial7.write(recv_buf, msg_buffer_len);
        Serial7.flush();
        Serial.println("Wrote "+String(type)+" message");
      }
      else if(!strcmp(type,type7_u)){ //Upstream Serial7
        Serial1.write(recv_buf7, msg_len7);
        Serial1.flush();
        Serial.println("Wrote "+String(type)+" data upstream");
      }
      #endif
//      else{
//        Serial.println("Unrecognized Type: "+String(type));
//      }
    }
  }
}
