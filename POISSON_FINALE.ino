#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_ADXL345_U.h>
//#include <Adafruit_GPS.h>
#include <Adafruit_BMP280.h>
#include <SD.h>


/*
  Sensors:
  - Adafruit_BMP280
  - Adafruit_BNO055
  - Adxl345
  - GPS
  - Resistor

  Output:
  - SD
  - RFM95
*/

#define ERROR_LED    10
#define INFO_LED     9

#define RFM95_INT     3
#define RFM95_RST     6
#define RFM95_CS      5

#define MOSI          11
#define MISO          12
#define SCK           13

#define BMP_CS        4
#define SD_CS         7

boolean info_led_state = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_BMP280 bmp(BMP_CS);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

typedef struct {
  unsigned int packet_number = millis();
  unsigned long time;
  float value1;
  float value2;
  float value3;
  float value4;
  float value5;
  float value6;
  float value7;
  float value8;
  float value9;
  float value10;
  float value11;
  float value12;
  float value13;
} packet_struct;

packet_struct bno_packet1;
packet_struct bno_packet2;
packet_struct adxl_bmp_packet;
imu::Vector<3> acc;
imu::Vector<3> linear;
imu::Vector<3> grav;
imu::Vector<3> mag;
imu::Vector<3> gyro;
imu::Quaternion quat;
sensors_event_t event;

File dataFile;

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(ERROR_LED, OUTPUT);
  pinMode(INFO_LED, OUTPUT);
  digitalWrite(INFO_LED, HIGH);
  delay(500);
  digitalWrite(INFO_LED, LOW);
  Serial.begin(9600);//DEBUG
  Serial.println("init");//DEBUG
  //SETUP INIT

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!( rf95.init() )) { // && bno.begin() && SD.begin(SD_CS)
    Serial.println("rfm init failed");//DEBUG
    fatal_error();
  }

  if (!( bmp.begin() )) { // && bno.begin() && SD.begin(SD_CS)
    Serial.println("bmp init failed");//DEBUG
    fatal_error();
  }

  Serial.println("init succeed");//DEBUG
  delay(1000);

  //SETUP TELEM
  rf95.setFrequency(433.0);
  rf95.setTxPower(1, false);

  //SETUP SD
  //dataFile = SD.open("DATA.txt", FILE_WRITE);

  //SETUP GPS
  /*
    GPS.begin(9600);
    //GPS.sendCommand("$PMTK251,57600*2C");
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    delay(1000);
    //GPS.begin(57600);
  */
  //SETUP BNO
  /*
    bno.setExtCrystalUse(true);
    blink_led(INFO_LED,4);
  */

  //SETUP Adxl345
  //accel.setRange(ADXL345_RANGE_16_G);
  //accel.setDataRate(ADXL345_DATARATE_100_HZ);

  //SETUP BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X8,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
}


void loop() {
  Serial.println("loop"); Serial.flush(); //DEBUG

  //BNO
  quat = bno.getQuat();
  mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  bno_packet1.packet_number = 0;

  bno_packet1.value1 = quat.x();
  bno_packet1.value2 = quat.y();
  bno_packet1.value3 = quat.z();
  bno_packet1.value4 = quat.w();

  bno_packet1.value5 = mag.x();
  bno_packet1.value6 = mag.y();
  bno_packet1.value7 = mag.z();

  bno_packet1.value8 = gyro.x();
  bno_packet1.value9 = gyro.y();
  bno_packet1.value10 = gyro.z();

  acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  bno_packet2.packet_number = 1;

  bno_packet2.value1 = linear.x();
  bno_packet2.value2 = linear.y();
  bno_packet2.value3 = linear.z();

  bno_packet2.value4 = grav.x();
  bno_packet2.value5 = grav.y();
  bno_packet2.value6 = grav.z();

  bno_packet2.value4 = acc.x();
  bno_packet2.value5 = acc.y();
  bno_packet2.value6 = acc.z();

  Serial.println("Sending packet"); Serial.flush(); //DEBUG
  send_binary_packet(bno_packet1, true);
  send_binary_packet(bno_packet2, true);


  //Adxl345

  Serial.println("adxl"); Serial.flush(); //DEBUG

  accel.getEvent(&event);
  adxl_bmp_packet.value1 = event.acceleration.x;
  adxl_bmp_packet.value2 = event.acceleration.y;
  adxl_bmp_packet.value3 = event.acceleration.z;
  adxl_bmp_packet.packet_number = 0;
  Serial.println("bmp"); Serial.flush(); //DEBUG
  //BMP280
  adxl_bmp_packet.value4 = bmp.readPressure();
  adxl_bmp_packet.value5 = bmp.readTemperature();

  Serial.println(adxl_bmp_packet.value4);
  Serial.println(adxl_bmp_packet.value5);
  Serial.println(event.acceleration.z);
  Serial.println("send bin"); Serial.flush(); //DEBUG
  send_binary_packet(adxl_bmp_packet, false);
  Serial.println("end bin"); Serial.flush(); //DEBUG
  //GPS
  //GPS.read();
  /*while(!GPS.newNMEAreceived()){
    GPS.read();
    }*/
  //Serial.println(GPS.lastNMEA());
  //send_string_packet(3,GPS.lastNMEA());
  //dataFile.close();
  invert_info_led();
  Serial.println("loopend"); Serial.flush(); //DEBUG
}

void send_string_packet(unsigned int packet_number, String input) {
  String string_packet = String(millis());
  string_packet = string_packet + String("|");
  string_packet = string_packet + input;

  char char_packet[60];
  string_packet.toCharArray(char_packet, 60);

  rf95.waitPacketSent();
  rf95.send((uint8_t *)char_packet, strlen(char_packet));
}


void send_binary_packet(packet_struct packet, boolean sd_write) {
  //rf95.waitPacketSent();
  uint8_t *char_packet;
  //memcpy(char_packet,&packet,sizeof(packet_struct));
  rf95.send( char_packet, sizeof(packet_struct));
  if (sd_write) {
    save_packet(packet);
  }
}

void blink_led(int pin, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(200);
    digitalWrite(pin, LOW);
    delay(200);
  }
}

void invert_info_led() {
  if (info_led_state) {
    digitalWrite(INFO_LED, LOW);
  } else {
    digitalWrite(INFO_LED, HIGH);
  }
}

void fatal_error() {
  while (1) {
    digitalWrite(ERROR_LED, HIGH);
    delay(200);
    digitalWrite(ERROR_LED, LOW);
    delay(200);
  }
}

void save_packet(packet_struct packet) {
  dataFile.print(packet.packet_number);
  dataFile.print("|");
  dataFile.print(packet.time);
  dataFile.print("|");

  if (packet.value1) {
    dataFile.print(packet.value1, 3);
    dataFile.print("|");
  }
  if (packet.value2) {
    dataFile.print(packet.value2, 3);
    dataFile.print("|");
  }
  if (packet.value3) {
    dataFile.print(packet.value3, 3);
    dataFile.print("|");
  }
  if (packet.value4) {
    dataFile.print(packet.value4, 3);
    dataFile.print("|");
  }
  if (packet.value5) {
    dataFile.print(packet.value5, 3);
    dataFile.print("|");
  }
  if (packet.value6) {
    dataFile.print(packet.value6, 3);
    dataFile.print("|");
  }
  if (packet.value7) {
    dataFile.print(packet.value7, 3);
    dataFile.print("|");
  }
  if (packet.value8) {
    dataFile.print(packet.value8, 3);
    dataFile.print("|");
  }
  if (packet.value9) {
    dataFile.print(packet.value9, 3);
    dataFile.print("|");
  }
  if (packet.value10) {
    dataFile.print(packet.value10, 3);
    dataFile.print("|");
  }
  if (packet.value11) {
    dataFile.print(packet.value11, 3);
    dataFile.print("|");
  }
  if (packet.value12) {
    dataFile.print(packet.value12, 3);
    dataFile.print("|");
  }
  if (packet.value13) {
    dataFile.print(packet.value13, 3);
    dataFile.print("|");
  }
  dataFile.println();
}
