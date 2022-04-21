//Interrupt Control for LeArm

//#include <MovingSteppersLib.h>
//#include <MotorEncoderLib.h>
#include <R2Protocol.h>

// use interrupts file for jetson to arduino communicaiton

#define MAX_ENCODER_VAL 16383

/* PROBLEMS LIST
  1. DO NOT HAVE TWO MOTORS HAVE SAME DIRECTION OR STEP PINS AS ANOTHER MOTOR EVERRRRR IT MESSES UP CODE
*/

// step pins 2
int s0 = 48;
int s1 = 35;
int s2 = 0;
int s3 = 0;
int s4 = 0;
int s5 = 0;
// direction pins
int d0 = 36;
int d1 = 34;
int d2 = 0;
int d3 = 0;
int d4 = 0;
int d5 = 0;
//chip select pins
int c0 = 10;
int c1 = 9;
int c2 = 0;
int c3 = 0;
int c4 = 0;
int c5 = 0;

//SoftwareSerial mySerial(19,18); // RX, TX

uint8_t send_buf[10];
int i = 0; 

// Jetson to Arduino Set up
uint16_t checksum;
char type[5];
uint8_t data[6]; 
uint32_t data_len = 6; 

uint8_t receive_buf[256];

volatile int counter = 0;
volatile int fill_serial_buffer = false;
void reset_input_buffer() {
  while (Serial1.available() > 0 ) Serial1.read();
  delay(100);
}

void setup()
{
  Serial.begin(9600); //Baud Rate
  Serial1.begin(9600); 
  Serial2.begin(115200);

  Serial.println("Hello World");
  delay(1000);
  reset_input_buffer();


}

// Arduino to Jetson R2
uint16_t servo_angles[] = {10, 20, 30, 40, 50, 60};
uint16_t new_servo_angles[] = {0};
uint8_t servo_anglesB8[12]; 
uint8_t send_buffer[256];
int k;

void update_servo_angles(){
  for (k =0; k <6; k++){
    //Send read command to LeArm
    
  }
}

void changeAngles(uint8_t data[]){
  for (i=0; i<6; i++){
    if (data[i] != servo_angles[i]){
      //Send change command to LeArm
      
    }
   }
}

void send(char type[5], const uint8_t* data, uint32_t data_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 256);
  Serial1.write(send_buffer, written);
  
  Serial.println("Bytes written: " + String(written));
//  for(int i=0; i <written; i++){
//    Serial.write(send_buffer[i]);
//  }
}

void loop()
{

  // Jetson to Arduino
  
   if (Serial1.available() > 28) {
      Serial.println("Bytes available: " + String(Serial1.available()));
      Serial1.readBytes(receive_buf, 28);
      for (i=0; i<28; i++) {
        Serial.println(receive_buf[i]);
      }
      //Serial.println(r2p_decode(receive_buf, 256, &checksum, type, data, &data_len));
      r2p_decode(receive_buf, 256, &checksum, type, data, &data_len);
      Serial.println(String(type));
      Serial.println("Checksum: " + String(checksum));
      Serial.println(data_len);
      Serial.println("done decode"); 

      Serial.println("Data");
      for (i=0; i<data_len; i++){
        Serial.println(data[i]); 
      }
     //Serial.println(data[1]);
     convert_b8_to_b16(new_servo_angles,data); //Converts to 16 bit versions of the servo angles
     changeAngles(new_servo_angles);
    } 
 
  // Arduino to Jetson   
 else{
    update_servo_angles();
    convert_b16_to_b8(servo_angles, servo_anglesB8, 6);
    send("prm", servo_anglesB8, 12, send_buffer);
    delay(100);
  }
                             
  
}




void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data) {
  int data_idx;
  for (int i=0; i < 12; i++) {
    data_idx = i / 2;
    if ( (i & 1) == 0) {
      // even
      data[data_idx] = databuffer[i] << 8;
    } else {
      // odd
      data[data_idx] |= databuffer[i];
    }
  }
}
void convert_b16_to_b8(int *databuffer, uint8_t *data, int len) {
  int data_idx1;
  int data_idx2;
  for (int i = 0; i < 2*len; i+=2) {
    data[i] = (databuffer[i/2] >> 8) & 255;
    data[i+1] = (databuffer[i/2]) & 255;
  }
}
