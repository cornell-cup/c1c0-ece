// Teensy Arbiter Test, Connected to two Arduino DUEs

// See the FAQ on the Cornell Cup Google Drive, located Here:
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit
// Author: Stefen Pegels, email sgp62@cornell.edu for contact

/*  I2C Info
 *  IMU Device: Adafruit BNO055
 *  Device address ->
 */

#include "R2Protocol.h"
/* ADDED IMU SETTING*/
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "imu.h"

// #define DEBUG

// Uncomment which serial ports are in use, make sure to define their parameters below
#define SER2
#define SER3
#define SER4
#define SER5
// #define SER6
//  #define SER7

// imu define
#define BNO055_SAMPLERATE_DELAY_MS (100)

// TEMPORARILY HERE, TRYING TO CLEAN UP SO NO OTHER DATA INVOLVED
#define MAX_BUFFER_SIZE 2048

// THIS IS IMPORTANT
#define IMU_DATA_LEN 6

// ORIGINALLY FOR SENSOR DATA BUFFER, DON'T THINK WE SHOULD NEED IT.
#define IMU_START TERABEE_DATA_LEN * 3 + LIDAR_DATA_LEN

// Need to declare these for each connected device

typedef struct
{
  HardwareSerial *serialobj;
  uint8_t STATE;
  uint8_t recv_buf[2048];
  uint32_t msg_len;
  const char type_d[5];
  const char type_u[5];
  uint16_t ser_count;
} serialBuffer;

// Jetson Serial Port
serialBuffer jetson_ser = {.serialobj = &Serial1, .STATE = 0, .recv_buf = {0}, .msg_len = MAX_BUFFER_SIZE, .type_d = "", .type_u = "", .ser_count = 0};

// Precise Arm Serial Port
#ifdef SER2
serialBuffer ser2 = {.serialobj = &Serial2, .STATE = 0, .recv_buf = {0}, .msg_len = 28, .type_d = "PRM", .type_u = "PRMR", .ser_count = 0};
#endif

// Sensor Serial Port
#ifdef SER3
serialBuffer ser3 = {.serialobj = &Serial3, .STATE = 0, .recv_buf = {0}, .msg_len = 1488, .type_d = "SENS", .type_u = "SNSR", .ser_count = 0};
#endif

// Strong Arm Serial Port
#ifdef SER4
serialBuffer ser4 = {.serialobj = &Serial4, .STATE = 0, .recv_buf = {0}, .msg_len = 29, .type_d = "STR", .type_u = "STRR", .ser_count = 0};
#endif

// Locomotion Serial Port
#ifdef SER5
serialBuffer ser5 = {.serialobj = &Serial5, .STATE = 0, .recv_buf = {0}, .msg_len = 19, .type_d = "loco", .type_u = "LOCR", .ser_count = 0};
#endif

// Not used
#ifdef SER6
serialBuffer ser6 = {.serialobj = &Serial6, .STATE = 0, .recv_buf = {0}, .msg_len = 1, .type_d = "", .type_u = "", .ser_count = 0};
#endif

#ifdef SER7
serialBuffer ser7 = {.serialobj = &Serial7, .STATE = 0, .recv_buf = {0}, .msg_len = 1, .type_d = "", .type_u = "", .ser_count = 0};
#endif

void serial_msg_complete(serialBuffer *ser)
{
  ser->ser_count = 0;
  ser->STATE = 0;
}

// Process jetson command and either send current buffer data upstream or send jetson message downstream
void process_jetson_command(serialBuffer *ser, char *type, uint32_t msg_buffer_len)
{
  // Check if head command which is locomotion head command
  char type_head[] = "head"; // Downstream type

  if (!strcmp(type, ser->type_d) || !strcmp(type, type_head))
  {
    // Send data downstream
    (*(ser->serialobj)).write(jetson_ser.recv_buf, msg_buffer_len);

    memset(ser->recv_buf, 0, MAX_BUFFER_SIZE);
    jetson_ser.STATE = 0;
    (*(ser->serialobj)).flush();

#ifdef DEBUG
    Serial.println("Wrote " + String(type) + " message downstream");
#endif
  }
  else if (!strcmp(type, ser->type_u))
  { // Upstream Serial4
    if (ser == &ser3)
      Serial1.write(ser->recv_buf, ser->msg_len + IMU_DATA_LEN + R2P_HEADER_SIZE);
    else
      Serial1.write(ser->recv_buf, ser->msg_len + R2P_HEADER_SIZE);
    memset(ser->recv_buf, 0, MAX_BUFFER_SIZE);
#ifdef DEBUG
    Serial.println("Wrote " + String(type) + " data upstream");
#endif
  }
}

// Interrupt handler for serial buffer
void serial_irq(serialBuffer *ser)
{
  if ((*(ser->serialobj)).available())
  {
    uint8_t val = (*(ser->serialobj)).read();
    ser->recv_buf[ser->ser_count] = val;
    (ser->ser_count)++;
    if (ser->ser_count < ser->msg_len)
    {
      if (ser->STATE == 0)
      {
        if (ser->recv_buf[0] == 0xa2)
          ser->STATE = 0xa2;
        else
          ser->ser_count = 0;
      }
      else if (ser->STATE == 0xa2)
      {
        if (ser->recv_buf[1] == 0xb2)
          ser->STATE = 0xb2;
        else
        {
          ser->ser_count = 0;
          ser->STATE = 0;
        }
      }
      else if (ser->STATE == 0xb2)
      {
        if (ser->recv_buf[2] == 0xc2)
          ser->STATE = 0xc2;
        else
        {
          ser->ser_count = 0;
          ser->STATE = 0;
        }
      }
    }
  }
}

void setup()
{
  Serial.println("Started");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);  // Serial monitor
  Serial1.begin(115200);

  imu_begin();
// These need to be declared in order to read and send messages to the respective devices
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

  Serial.println("Finished setup");
}

inline bool full_jetson_packet_received()
{
  return jetson_ser.recv_buf[jetson_ser.ser_count - 3] == 0xd2 && jetson_ser.recv_buf[jetson_ser.ser_count - 2] == 0xe2 && jetson_ser.recv_buf[jetson_ser.ser_count - 1] == 0xf2;
}

#ifdef SER2
void serialEvent2()
{
  serial_irq(&ser2);
  if (ser2.ser_count == ser2.msg_len)
  {
    serial_msg_complete(&ser2);
  }
}
#endif

#ifdef SER3
void serialEvent3()
{
  serial_irq(&ser3);
  if (ser3.ser_count == ser3.msg_len)
  {
#ifdef DEBUG
    Serial.println("Sensor Data Received");
#endif

    uint32_t msg_buffer_len3;
    uint16_t checksum3;
    char type3[5];
    uint8_t msg3[MAX_BUFFER_SIZE] = {0};

    // decoding message and adding IMU data to it
    uint32_t data_length;
    r2p_decode(ser3.recv_buf, ser3.ser_count, &checksum3, type3, msg3, &data_length);
    memset(ser3.recv_buf, 0, MAX_BUFFER_SIZE);
    uint16_t imu_data[3] = {0};
    imu_get_data(imu_data);
    // if (imu_data[1] == 0) {
    //   Serial.println("IMU Data[1] is zero");
    // }
    convert_b16_to_b8(imu_data, msg3 + ser3.msg_len, IMU_DATA_LEN / 2);
    r2p_encode(type3, msg3, ser3.msg_len + IMU_DATA_LEN, ser3.recv_buf, ser3.msg_len + IMU_DATA_LEN + R2P_HEADER_SIZE);

    serial_msg_complete(&ser3);
  }
}
#endif

#ifdef SER4
void serialEvent4()
{
  serial_irq(&ser4);
  if (ser4.ser_count == ser4.msg_len)
  {
    serial_msg_complete(&ser4);
  }
}
#endif

#ifdef SER5
void serialEvent5()
{
  serial_irq(&ser5);
  if (ser5.ser_count == ser5.msg_len)
  {
    serial_msg_complete(&ser5);
  }
}
#endif

#ifdef SER6
void serialEvent6()
{
  serial_irq(&ser6);
  if (ser6.ser_count == ser6.msg_len)
  {
    serial_msg_complete(&ser6);
  }
}
#endif

#ifdef SER7
void serialEvent7()
{
  serial_irq(&ser7);
  if (ser7.ser_count == ser7.msg_len)
  {
    serial_msg_complete(&ser7);
  }
}
#endif
// Receives variable length messages from Jetson, end_cmp() detects if we have a valid packet
void serialEvent1()
{
  if (jetson_ser.STATE != 0xf2)
  {
    serial_irq(&jetson_ser);
    if (full_jetson_packet_received())
      jetson_ser.STATE = 0xf2;
  }
}

void loop()
{
  // Successfully found a packet from the jetson
  // Serial.println("Waiting");
  // delay(100);
  // uint16_t imu_data[3] = {0};
  // imu_get_data(imu_data);
  // delay(100);

  if (jetson_ser.STATE == 0xf2)
  {
#ifdef DEBUG
    Serial.println("Received data from Jetson");
#endif
    uint16_t checksum;
    char type[5];
    uint8_t msg[MAX_BUFFER_SIZE] = {0};
    uint32_t msg_len;

    r2p_decode(jetson_ser.recv_buf, jetson_ser.ser_count, &checksum, type, msg, &msg_len);

#ifdef SER2
    process_jetson_command(&ser2, type, jetson_ser.ser_count);
#endif
#ifdef SER3
    process_jetson_command(&ser3, type, jetson_ser.ser_count);
#endif
#ifdef SER4
    process_jetson_command(&ser4, type, jetson_ser.ser_count);
#endif
#ifdef SER5
    process_jetson_command(&ser5, type, jetson_ser.ser_count);
#endif
#ifdef SER6
    process_jetson_command(&ser6, type, jetson_ser.ser_count);
#endif
#ifdef SER7
    process_jetson_command(&ser7, type, jetson_ser.ser_count);
#endif

    serial_msg_complete(&jetson_ser);
  }
}
