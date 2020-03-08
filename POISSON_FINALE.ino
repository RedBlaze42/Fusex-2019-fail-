#include <SPI.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>
#include <RH_RF95.h>
#include <RadioHead.h>

/*
  Sensors:
  - Adafruit_BMP280
  - Adafruit_BN
init
SD oksetup
Sending
save_packet
fin_bno
init
O055
  - Adxl345
  - GPS
  - Resistor

  Output:
  - SD
  - RFM95
*/

#define ERROR_LED    9
#define INFO_LED     10

#define RFM95_INT     3
#define RFM95_RST     6
#define RFM95_CS      5

#define MOSI          11
#define MISO          12
#define SCK           13

#define BMP_CS        4
#define SD_CS         7

#define GPS_REFRESH_RATE 10000

//boolean info_led_state = false;

RH_RF95 rf95(RFM95_CS,RFM95_INT);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP280 bmp(BMP_CS);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_GPS GPS(&Serial2);

typedef struct {
  unsigned int packet_number;
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

packet_struct packet;
//packet_struct bno_packet2;
//packet_struct adxl_bmp_packet;
//packet_struct gps_packet;

imu::Vector<3> vect;
imu::Quaternion quat;
sensors_event_t event;

long last_gps_time=-100000;
String last_gps_value="None";
String gps_value="None";
File dataFile;

void setup() {
  Serial.begin(9600);
  Serial.println("init");
  pinMode(SD_CS, OUTPUT);
  pinMode(INFO_LED,OUTPUT);
  if (!SD.begin(SD_CS)) {
    Serial.print("pb SD on CS ");
    Serial.println(SD_CS);
    error();
  }
  Serial.print("SD ok");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  /*if (!( rf95.init() && bno.begin() && bmp.begin() && SD.begin(SD_CS) )) {
    Serial.println("pb");
    while(1);
  }*/

  delay(500);
  
  
  if(!bmp.begin()){
    Serial.println("pb bmp");
    while(1);
  }
  
  if(!bno.begin()){
    Serial.println("pb bno");
    error();
  }
  
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if(!rf95.init()){
    Serial.println("pb rf95");
    error();
  }

  delay(1000);

  //SETUP TELEM
  rf95.setFrequency(433.0);
  rf95.setTxPower(23, false);
//  rf95.setLowDatarate();


  //SETUP SD
  //SETUP GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  //SETUP BNO
  
  bno.setExtCrystalUse(true);
    
  //SETUP Adxl345
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);

  //SETUP BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X1,
                  Adafruit_BMP280::SAMPLING_X8, 
                  Adafruit_BMP280::FILTER_X8,
                  Adafruit_BMP280::STANDBY_MS_1);
           Serial.println("setup"); Serial.flush();
  digitalWrite(INFO_LED,HIGH);
}


void loop() {
  
  //BNO
  quat = bno.getQuat();
  vect = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  packet.time=millis();

  packet.value5 = vect.x();
  packet.value6 = vect.y();
  packet.value7 = vect.z();
  
  vect = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  packet.packet_number = 0;

  packet.value1 = quat.x();
  packet.value2 = quat.y();
  packet.value3 = quat.z();
  packet.value4 = quat.w();

  packet.value8 = vect.x();
  packet.value9 = vect.y();
  packet.value10 = vect.z();

  send_binary_packet(packet, true);
  packet.time=millis();
  vect = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  packet.value4 = vect.x();
  packet.value5 = vect.y();
  packet.value6 = vect.z();
  
  vect = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  packet.value1 = vect.x();
  packet.value2 = vect.y();
  packet.value3 = vect.z();
  
  vect = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  packet.value4 = vect.x();
  packet.value5 = vect.y();
  packet.value6 = vect.z();

  packet.packet_number = 1;
Serial.println("fin_bno"); Serial.flush();
  send_binary_packet(packet, true);
Serial.println("fin_telem"); Serial.flush();
  //Adxl345


  accel.getEvent(&event);
  packet.time=millis();
  packet.value1 = event.acceleration.x;
  packet.value2 = event.acceleration.y;
  packet.value3 = event.acceleration.z;
  packet.packet_number = 2;
  //BMP280-
  packet.value4 = bmp.readPressure();
  packet.value5 = bmp.readTemperature();

  //ANALOG
  packet.value6 = analogRead(A0);
  packet.value7 = analogRead(A1);
  packet.value8 = analogRead(A2);
  packet.value9 = analogRead(A3);
  Serial.println("fin_bmp"); Serial.flush();
  send_binary_packet(packet, false);

  //GPS
  /*
  if(millis()-last_gps_time>GPS_REFRESH_RATE){
    GPS.read();
    while(!GPS.newNMEAreceived()){
      GPS.read();
    }
    gps_value=GPS.lastNMEA();
    gps_value=gps_value.substring(0,63);
    gps_value.replace("\n","");
    gps_value=gps_value+String("\n");
    last_gps_value=gps_value;
    last_gps_time=millis();
    gps_value=gps_value+String("|V");
  }else{
    gps_value=last_gps_value+String("|F");
  }
  send_string_packet(3,gps_value);*/
}


void send_string_packet(unsigned int packet_number, String input) {
  String string_packet = String(packet_number) + String("|");
  string_packet = string_packet + String(millis());
  string_packet = string_packet + String("|");
  string_packet = string_packet + input;

  char char_packet[string_packet.length()];
  string_packet.toCharArray(char_packet, string_packet.length());

  Serial.write(char_packet);
  rf95.waitPacketSent();
  Serial.println("");
  Serial.println(sizeof(char_packet));
  Serial.println(char_packet);
  Serial.println("");
  rf95.send(char_packet,sizeof(char_packet));
  dataFile = SD.open("DATA.txt", FILE_WRITE);
  dataFile.println(string_packet);
  dataFile.close();
}


void send_binary_packet(packet_struct packet, boolean sd_write) {
  rf95.waitPacketSent();
  Serial.println("Sending"); Serial.flush();
  uint8_t char_packet[sizeof(packet)]; 
  memcpy(char_packet,&packet,sizeof(packet));
  //uint8_t *char_packet="test";
  if(!rf95.waitPacketSent(750)){
    rf95.setModeIdle();
    Serial.println("ERROR################################################################################");
  }
  
  rf95.send(char_packet,sizeof(char_packet));
  save_packet(packet);
}

void save_packet(packet_struct packet) {
  Serial.println("save_packet"); Serial.flush();
  dataFile = SD.open("DATA.txt", FILE_WRITE);
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
  dataFile.close();
 
}

void error(){
  while(1){
    digitalWrite(INFO_LED,HIGH);
    delay(350);
    digitalWrite(INFO_LED,LOW);
    delay(350);
    Serial.println("ERROR");
  }
}
