#include <Arduino.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_NeoPixel.h>
#include "SD.h"
#include <SPI.h>
#include <math.h>
//#include <RH_RF95.h>
#include <LoRa.h>
#include <Ticker.h>
#include <Wire.h>

// Serial config
int decimal_places = 4;
String data_seperator = "\t";

/*
// NeoPixel
#define PIN 0
#define NUMPIXELS 1
Adafruit_NeoPixel LED(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
*/

// File system setup
#define SD_CS_PIN 14
String FILE_NAME = "/data.csv";
File myFile;

char data_char[200];

struct data_struct {
  float sec;
  float delta_millis;
  float kalman_x;
  float kalman_y;
  float kalman_z;
  float acc_x;
  float acc_y;
  float acc_z;
  float vel_x;
  float vel_y;
  float vel_z;
  float q0;
  float q1;
  float q2;
  float q3;
  float GPS_lat;
  float GPS_lon;
  float GPS_m_x;
  float GPS_m_y;
  float BMP_alt;
};

const int dataArraySize = 50;
data_struct data[dataArraySize];
int writing_index;

const int buttonPin = 38;
volatile bool button_pressed = false;
bool button_active = true;

// Radio setup
#define RFM95_CS 25
#define RFM95_RST 33
#define RFM95_INT 13

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.69

// Singleton instance of the radio driver
//RH_RF95 rf95(RFM95_CS, RFM95_INT);

String packet_data_seperator = "\t";
int radio_timer;
int radio_freq = 1;
int radio_packet_num;


// BMP Setup
#define BMP_CS 4
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

const float BMP_Stdev = 0.5;
float BMP_alt, last_BMP_alt, BMP_alt_vel;
int BMP_freq = 10;
int BMP_timer;

// Thermistor Setup
int analogValue;
float voltage;
float temperature;
const float r_25 = 2252;
const float r_fixed = 3000;

const double a = 0.0033540154;
const double b = 0.00025627725;
const double c = 0.000002082921;
const double d = 0.000000073003206;

// BNO Setup
int16_t packetnum = 0;

#define BNO08X_CS 15
#define BNO08X_INT 27
#define BNO08X_RESET 32

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Interrupt
const float IMU_INTERVAL = 0.005;
Ticker tickerIMU;

// Variables
float acc_x, acc_y, acc_z, vel_x, vel_y, vel_z, pos_x, pos_y, pos_z, q0, q1, q2, q3;
float x_color_factor, y_color_factor, z_color_factor;
float delta_micros;
unsigned long last_micros;
bool FlipFlop;

float kalman_offset_x, kalman_offset_y, kalman_offset_z;

// GPS Setup
#define PI 3.1415926535897932384626433832795

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

const float GPS_Stdev = 3;

unsigned long last_millis;

struct DeltaMetersOutput {
  float x;
  float y;
};

float GPS_lat, GPS_lon, GPS_ori_lat, GPS_ori_lon, GPS_speed, GPS_angle, GPS_delta_meter_x, GPS_delta_meter_y, GPS_vel_x, GPS_vel_y;

const float radius_equa = 6378137;
const float radius_pole = 6371001;

// Status LED pin
#define STATUS_LED 22

// Kalman filter parameters
const float IMU_R = pow(GPS_Stdev, 2);  // IMU Measurement noise covariance
const float BMP_R = pow(BMP_Stdev, 2);  // BMP Measurement noise covariance
const float Q = 0.01;                   // Process noise covariance

float IMU_K = 0;  // IMU Inital Kalman gain
float IMU_P = 0;  // IMU Initial error covariance

float BMP_K = 0;  // BMP Inital Kalman gain
float BMP_P = 0;  // BMP Initial error covariance

float kalman_x = 0;  // Inital state estimate along x-axis
float kalman_y = 0;  // Inital state estimate along y-axis
float kalman_z = 0;  // Inital state estimate along z-axis

const float CORRECTION_MAGNITUDE = 0.1;  // (m/s)/s

// Enable components
bool ENABLE_SD =      true;
bool ENABLE_Radio =   true;

bool ENABLE_BMP =     true;
bool ENABLE_TRM =     true;
bool ENABLE_BNO =     true;
bool ENABLE_GPS =     true;

// Multithreading
//TaskHandle_t DHCore; // Data-Handling Core

void setup() {
  Serial.begin(115200);

  // NeoPixel
  //LED.begin();
  //LED.show();

  // Status LED
  pinMode(STATUS_LED, OUTPUT);

  if (ENABLE_SD) {
    setupSD();
    writeHeader();
  }
  if (ENABLE_Radio) {
    setupRadio();
  }

  if (ENABLE_BMP) {
    setupBMP();
  }
  if (ENABLE_TRM) {
    setupTRM();
  }
  if (ENABLE_BNO) {
    setupBNO();
  }
  if (ENABLE_GPS) {
    setupGPS();
  }

  resetKalmanState();
  
  if (ENABLE_SD) {
    deleteData();
  }

  setupBlink();

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, FALLING);
  Serial.println("Delay start");
  delay(3000);
  Serial.println("Delay end");

  xTaskCreatePinnedToCore(
    DHTask,
    "DataHandling",
    10000,
    NULL,
    1,
    NULL, // &DHCore
    1);

  if (ENABLE_BNO) {
    tickerIMU.attach(IMU_INTERVAL, IMU_INTERRUPT);
  }
}

void setupSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    errorBlink("SD");
  }
}

void writeHeader() {
  char* header = "Seconds [s], Kalman x [m], Kalman y [m], Kalman z [m], Acceleration x [m/s^2], Acceleration y [m/s^2], Acceleration z [m/s^2], Velocity x [m/s], Velocity y [m/s], Velocity z [m/s], q0, q1, q2, q3, Latitude [degrees], Longitude [degrees], GPS Delta x [m], GPS Delta y [m], BMP Altitude [m]";
  appendFile(header);
}

void setupRadio() {
  /*
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    errorBlink("Radio");
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Set frequency failed");
    errorBlink("Freq");
  }

  //rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  Serial.print("Set ModemConfig to: Bw500Cr45Sf128");
  rf95.setSpreadingFactor(10);
  //rf95.setPreambleLength(4);
  //rf95.setPayloadCRC(false);
  rf95.setTxPower(23, false);
  */
  
  Serial.println("LoRa Sender");
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  if (!LoRa.begin(434.69E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  } else {
    Serial.println("Starting LoRa successful!");
  }

  //LoRa.setSpreadingFactor(12);
  //LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(20);
}

void setupBMP() {
  if (!bmp.begin_SPI(BMP_CS)) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    errorBlink("BMP");
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void setupTRM() {
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
}

void setupBNO() {
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Couldn't initialize BNO085!");
    errorBlink("BNO");
  }
  setReports();

  delay(100);

  if (bno08x.wasReset()) {
    setReports();
    errorBlink("BNO was reset in setup");
    return;
  }
}

void setupGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
}

void setupBlink() {
  // Status LED
  digitalWrite(STATUS_LED, HIGH);

  /*
  // NeoPixel
  for (int i = 0; i < 3; i++) {
    LED.setPixelColor(0, LED.Color(255, 255, 255));
    LED.show();
    delay(50);
    LED.setPixelColor(0, LED.Color(0, 0, 0));
    LED.show();
    delay(100);
  }
  */
}

void errorBlink(String message) {  
  // Status LED
  digitalWrite(STATUS_LED, LOW);

  Serial.print("ERROR: ");
  Serial.println(message);

  delay(5000);

  /*
  // NeoPixel
  LED.setPixelColor(0, LED.Color(255, 0, 0));
  LED.show();
  delay(200);
  LED.setPixelColor(0, LED.Color(0, 0, 0));
  LED.show();
  delay(200);
  */

  //while (1) {}
}

void appendFile(char* message) {
  myFile = SD.open(FILE_NAME, FILE_APPEND);
  if (myFile) {
    myFile.println(message);
    myFile.flush();
    myFile.close();
  } else {
    Serial.print("\t");
    Serial.println("Error opening file for writing.");
    errorBlink("SD Append");
  }
}

void checkCommand() {
  if (!button_pressed) { return; }

  button_active = false;
  //LED.setPixelColor(0, LED.Color(255, 255, 255));
  //LED.show();
  delay(1000);
  //LED.setPixelColor(0, LED.Color(0, 0, 0));
  //LED.show();
  button_active = true;

  Serial.println();
  Serial.println("Waiting for command...");
  Serial.println();

  while (button_pressed) {
    if (Serial.available() > 0) {
      char receivedChar = Serial.read();

      // Perform actions based on the received command
      switch (receivedChar) {
        case 'k':
          resetKalmanState();
          break;

        case 'r':
          readData();
          break;

        case 'd':
          deleteData();
          break;

        default:
          Serial.print("\t");
          Serial.println("Unknown command received.");
          Serial.println();
          errorBlink("Unknown command received");
          break;
      }

      if (button_pressed) {
        Serial.println();
        Serial.println("Waiting for command...");
      }

      Serial.println();
      serialFlush();
    }
  }
  Serial.println("Command canceled");
  Serial.println();
}

void resetKalmanState() {
  kalman_offset_x = kalman_x;
  kalman_offset_y = kalman_y;
  kalman_offset_z = kalman_z;

  kalman_x -= kalman_offset_x;
  kalman_y -= kalman_offset_y;

  Serial.println("Kalman State Reset!");
}

void readData() {
  myFile = SD.open(FILE_NAME);
  if (myFile) {
    //LED.setPixelColor(0, LED.Color(0, 0, 255));
    //LED.show();
    digitalWrite(STATUS_LED, LOW);
    while (myFile.available() > 0) {
      Serial.println(myFile.readStringUntil('\n'));
    }
    myFile.close();
    digitalWrite(STATUS_LED, HIGH);

    //LED.setPixelColor(0, LED.Color(0, 0, 0));
    //LED.show();
    delay(500);
  } else {
    Serial.print("\t");
    Serial.println("Error opening file for reading.");
    errorBlink("SD Read");
  }
}

void deleteData() {
  if (SD.exists(FILE_NAME)) {
    Serial.print("\t");
    Serial.print("Deleting file in...");
    delay(1000);
    for (int i = 3; i > 0; i--) {
      Serial.print(" " + String(i) + "...");
      //LED.setPixelColor(0, LED.Color(255, 0, 0));
      //LED.show();
      digitalWrite(STATUS_LED, HIGH);
      delay(500);
      //LED.setPixelColor(0, LED.Color(0, 0, 0));
      //LED.show();
      digitalWrite(STATUS_LED, LOW);
      delay(500);
    }
    SD.remove(FILE_NAME);
    Serial.println(" File deleted!");
  } else {
    Serial.print("\t");
    Serial.println("[" + FILE_NAME + "] not found");
    errorBlink("SD Delete");
  }
}

void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

void setReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable linear acceleration");
    errorBlink("BNO Rotation Vector");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("Could not enable linear acceleration");
    errorBlink("BNO Linear Acceleration");
  }
}

void rotateVectorWithQuaternion(float *vector, float *quaternion) {
  // Convert the quaternion to a rotation matrix
  float rotMatrix[3][3];
  rotMatrix[0][0] = 1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
  rotMatrix[0][1] = 2 * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);
  rotMatrix[0][2] = 2 * (quaternion[1] * quaternion[3] + quaternion[0] * quaternion[2]);
  rotMatrix[1][0] = 2 * (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]);
  rotMatrix[1][1] = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[3] * quaternion[3]);
  rotMatrix[1][2] = 2 * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]);
  rotMatrix[2][0] = 2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
  rotMatrix[2][1] = 2 * (quaternion[2] * quaternion[3] + quaternion[0] * quaternion[1]);
  rotMatrix[2][2] = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);

  // Multiply the rotation matrix with the vector to get the rotated vector
  float result[3] = { 0 };
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result[i] += rotMatrix[i][j] * vector[j];
    }
  }

  // Update the input vector array with the rotated values (switch x and y components)
  vector[0] = result[0];
  vector[1] = result[1];
  vector[2] = result[2];
}

float convertCords(float degrees_minutes) {
  float degrees = floor(degrees_minutes / 100);
  float minutes = degrees_minutes - (degrees * 100);
  float decimal_degrees = degrees + (minutes / 60);

  return decimal_degrees;
}

DeltaMetersOutput calculateDeltaMeter(float lat1, float lon1, float lat2, float lon2, float alt) {
  float delta_lat = lat2 - lat1;
  float delta_lon = lon2 - lon1;

  float average_lat = (lat1 + lat2) / 2;

  float calc_delta_meter_x = (delta_lon / 360) * cos(radians(average_lat)) * ((radius_pole + alt) * 2 * PI);
  float calc_delta_meter_y = (delta_lat / 360) * ((radius_equa + alt) * 2 * PI);

  return { calc_delta_meter_x, calc_delta_meter_y };
}

void KalmanFilter(float &x, float v, float z, float &K, float R, float &P, int index) {
  // Predict
  float pred_x = x + v * (delta_micros * 0.000001);
  float pred_P = P + Q;

  // Update
  float innovation = z - pred_x;
  float S = pred_P + R;
  K = pred_P / S;
  x = pred_x + K * innovation;
  P = (1 - K) * P;

  /*
  if (index == 2) {
    Serial.print("  ");
    Serial.println(innovation);
  } else {
    Serial.print("  ");
    Serial.print(innovation);
  }
  */
}

void correctionAlgorithm(float &vel, float target_vel, float delta_micros) {
  if (vel > target_vel) {
    vel -= CORRECTION_MAGNITUDE * (delta_micros * 0.000001);
  } else {
    vel += CORRECTION_MAGNITUDE * (delta_micros * 0.000001);
  }
}

void buttonInterrupt() {
  if (button_active) {
    if (button_pressed) {
      button_pressed = false;
    } else {
      button_pressed = true;
    }
  }
}

void IMU_INTERRUPT() {
  if (button_pressed) { return; }

  //Serial.print("BNO running on core");
  //Serial.println(xPortGetCoreID());

  if (ENABLE_BNO) {
    if (bno08x.wasReset()) {
      setReports();
      errorBlink("BNO was reset");
      return;
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
      if (FlipFlop) {
        if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
          acc_x = sensorValue.un.linearAcceleration.x;
          acc_y = sensorValue.un.linearAcceleration.y;
          acc_z = sensorValue.un.linearAcceleration.z;

          float vector[3] = { acc_x, acc_y, acc_z };
          float quaternion[4] = { q0, q1, q2, q3 };
          rotateVectorWithQuaternion(vector, quaternion);

          delta_micros = micros() - last_micros;
          last_micros = micros();

          vel_x += vector[0] * (delta_micros * 0.000001);
          vel_y += vector[1] * (delta_micros * 0.000001);
          vel_z += vector[2] * (delta_micros * 0.000001);     

          float delta_since_gps = (millis() - last_millis) * 0.001;

          KalmanFilter(kalman_x, vel_x, GPS_delta_meter_x + delta_since_gps * GPS_vel_x, IMU_K, IMU_R, IMU_P, 0);
          KalmanFilter(kalman_y, vel_y, GPS_delta_meter_y + delta_since_gps * GPS_vel_y, IMU_K, IMU_R, IMU_P, 1);
          KalmanFilter(kalman_z, vel_z, BMP_alt, BMP_K, BMP_R, BMP_P, 2);

          correctionAlgorithm(vel_x, GPS_vel_x, delta_micros);
          correctionAlgorithm(vel_y, GPS_vel_y, delta_micros);
          correctionAlgorithm(vel_z, 0, delta_micros);

          //  Visualize Accelerometer Vector with LED
          //x_color_factor = 1 / (1 + (pow(vector[0], 2) / 50));
          //y_color_factor = 1 / (1 + (pow(vector[1], 2) / 50));
          //z_color_factor = 1 / (1 + (pow(vector[2], 2) / 50));

          //LED.setPixelColor(0, LED.Color(255 * (1 - x_color_factor), 255 * (1 - y_color_factor), 255 * (1 - z_color_factor)));
          //LED.show();
        
          data[writing_index].sec = float(micros()) * 0.000001;
          data[writing_index].delta_millis = delta_micros * 0.001;
          data[writing_index].kalman_x = kalman_x;
          data[writing_index].kalman_y = kalman_y;
          data[writing_index].kalman_z = kalman_z;
          data[writing_index].acc_x = acc_x;
          data[writing_index].acc_y = acc_y;
          data[writing_index].acc_z = acc_z;
          data[writing_index].vel_x = vel_x;
          data[writing_index].vel_y = vel_y;
          data[writing_index].vel_z = vel_z;
          data[writing_index].q0 = q0;
          data[writing_index].q1 = q1;
          data[writing_index].q2 = q2;
          data[writing_index].q3 = q3;
          data[writing_index].GPS_lat = GPS_lat;
          data[writing_index].GPS_lon = GPS_lon;
          data[writing_index].GPS_m_x = GPS_delta_meter_x + delta_since_gps * GPS_vel_x;
          data[writing_index].GPS_m_y = GPS_delta_meter_y + delta_since_gps * GPS_vel_y;
          data[writing_index].BMP_alt = BMP_alt;

          //Serial.println(writing_index);

          writing_index++;

          /*
          Serial.print(kalman_x, decimal_places);
          Serial.print(data_seperator);
          Serial.print(kalman_y, decimal_places);
          Serial.print(data_seperator);
          Serial.println(kalman_z, decimal_places);

          Serial.print(data_seperator);
          Serial.print(GPS_delta_meter_x + delta_since_gps * GPS_vel_x);
          Serial.print(data_seperator);
          Serial.print(GPS_delta_meter_y + delta_since_gps * GPS_vel_y);
          Serial.print(data_seperator);
          Serial.print(vel_x, decimal_places);
          Serial.print(data_seperator);
          Serial.print(vel_y, decimal_places);
          Serial.print(data_seperator);
          Serial.print(vel_z, decimal_places);
          Serial.print(data_seperator);
          Serial.print(GPS_vel_x, decimal_places);
          Serial.print(data_seperator);
          Serial.print(GPS_vel_y, decimal_places);
          Serial.print(data_seperator);
          Serial.print(GPS_speed, decimal_places);
          Serial.print(data_seperator);
          Serial.println(GPS_angle, decimal_places);
          */

          FlipFlop = false;
        }
      } else {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
          q0 = sensorValue.un.rotationVector.real;
          q1 = sensorValue.un.rotationVector.i;
          q2 = sensorValue.un.rotationVector.j;
          q3 = sensorValue.un.rotationVector.k;
          FlipFlop = true;
        }
      }
    }
  }
}

float temp(float voltage) {
  float r_temp = voltage * r_fixed / (3.3 - voltage);

  return (1 / (a + b * log(r_temp / r_25) + c * pow(log(r_temp / r_25), 2) + d * pow(log(r_temp / r_25), 3)) - 273.15);
}

void checkGPSSentence() {
  if (button_pressed) { return; }

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  } else {
    return;
  }

  Serial.print("New sentence recieved: ");

  if (GPS_ori_lat == 0 && GPS_ori_lon == 0) {
    GPS_ori_lat = GPS_lat;
    GPS_ori_lon = GPS_lon;
  }

  GPS_lat = convertCords(GPS.latitude);
  GPS_lon = convertCords(GPS.longitude);

  Serial.print(GPS_lat, 6);
  Serial.print(", ");
  Serial.println(GPS_lon, 6);

  GPS_speed = GPS.speed * 0.51444444444444;
  GPS_angle = GPS.angle;

  GPS_vel_x = GPS_speed * cos(GPS_angle * PI / 180);
  GPS_vel_y = GPS_speed * sin(GPS_angle * PI / 180);

  DeltaMetersOutput delta_meters_output = calculateDeltaMeter(GPS_lat, GPS_lon, GPS_ori_lat, GPS_ori_lon, BMP_alt);

  if (sqrt(pow(delta_meters_output.x, 2) + pow(delta_meters_output.y, 2)) < 10) {
    GPS_delta_meter_x = delta_meters_output.x;
    GPS_delta_meter_y = delta_meters_output.y;
  }

  if (abs(GPS_delta_meter_x) > 100000) {
    GPS_delta_meter_x = 0;
    GPS_delta_meter_y = 0;
  }

  last_millis = millis();
}

void sendRadioPacket() {
  /*
  char radiopacket[3] = "Hi";

  /*
  dtostrf(lat, 7, 4, radiopacket + strlen(radiopacket));
  strcat(radiopacket, packet_data_seperator);
  dtostrf(lon, 7, 4, radiopacket + strlen(radiopacket));
  strcat(radiopacket, packet_data_seperator);
  dtostrf(alt, 5, 2, radiopacket + strlen(radiopacket));

  radiopacket[2] = 0;
  Serial.println("Sending: " + String(radiopacket));
  rf95.send((uint8_t *)radiopacket, 3);
  Serial.println("Sent: " + String(radiopacket));
  */

  LoRa.beginPacket();
  LoRa.print(radio_packet_num);
  LoRa.print(",");
  LoRa.print(BMP_alt);
  LoRa.endPacket(true);

  radio_packet_num++;
  
  /*
  //String data = "Temp: " + String(temperature) + "  Alt: " + String(BMP_alt) + " Lat: " + String(GPS_lat, 6) + " Lon: " + String(GPS_lon, 6);
  String data = "test";

  char sensorData[data.length() + 1];

  data.toCharArray(sensorData, sizeof(sensorData));

  Serial.println("Sending: " + data);
  rf95.send((u_int8_t *)sensorData, sizeof(sensorData));
  //rf95.waitPacketSent();
  Serial.println("Sent: " + data);
  //delay(200);
  */
}

void DHTask( void * parameter ) {
  for(;;) {
    //Serial.println(uxTaskGetStackHighWaterMark(NULL));

    checkCommand();
    if (ENABLE_GPS) {
      checkGPSSentence();
    }

    if (millis() - radio_timer > 1000 / radio_freq) {
      radio_timer = millis();
      if (ENABLE_Radio) {
        digitalWrite(STATUS_LED, LOW);

        float millis_1 = micros();
        sendRadioPacket();

        // send packet
        Serial.print("Radio running on core");
        Serial.println(xPortGetCoreID());

        float millis_2 = micros();
        Serial.print("Radio sent and took ");
        Serial.print(millis_2 - millis_1);
        Serial.println(" microseconds");

        digitalWrite(STATUS_LED, HIGH);
      }

      if (ENABLE_TRM) {
        analogValue = analogRead(26);
        voltage = 3.3 * analogValue / 4096;
        temperature = temp(voltage);
        Serial.print("Temp: ");
        Serial.println(temperature);
      }
    }

    if (writing_index >= dataArraySize - 25) {
      if (ENABLE_SD) {
        int readRange = writing_index;

        myFile = SD.open(FILE_NAME, FILE_APPEND);

        if (myFile) {
          for (int j = 0; j < readRange; j++) {
            sprintf(data_char, "%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.3f,%.3f,%.3f", 
              data[0].sec, 
              data[0].delta_millis, 
              data[0].kalman_x, 
              data[0].kalman_y, 
              data[0].kalman_z, 
              data[0].acc_x, 
              data[0].acc_y, 
              data[0].acc_z, 
              data[0].vel_x, 
              data[0].vel_y, 
              data[0].vel_z, 
              data[0].q0, 
              data[0].q1, 
              data[0].q2, 
              data[0].q3, 
              data[0].GPS_lat, 
              data[0].GPS_lon, 
              data[0].GPS_m_x, 
              data[0].GPS_m_y, 
              data[0].BMP_alt);

            myFile.println(data_char);
            myFile.flush();

            for (int k = 0; k < dataArraySize - 1; k++) {
              data[k] = data[k + 1];
            }
            writing_index--;
          }
          myFile.close();
          //Serial.println("Write complete");
          //Serial.println();
        } else {
          Serial.println("Write failed");
          Serial.println();
        }
      }
    }
  }
}

void loop() {
  if (millis() - BMP_timer > 1000 / BMP_freq) {
    if (ENABLE_BMP) {
      BMP_timer = millis();
      BMP_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      BMP_alt -= kalman_offset_z;  
      BMP_alt_vel = (BMP_alt - last_BMP_alt) * 10;
      last_BMP_alt = BMP_alt;

      Serial.print("Altitude is: ");
      Serial.println(BMP_alt);
    }
  }
}