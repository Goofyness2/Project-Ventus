#include <Arduino.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include "FS.h"
#include "SPIFFS.h"
#include "SD.h"
#include <SPI.h>
#include <math.h>
//#include <RH_RF95.h>
#include <LoRa.h>
#include <Linear_Algebra.h>  // Include the library first
#include <include/linalg/HouseholderQR.hpp>
#include <include/linalg/Matrix.hpp>
#include <include/linalg/Arduino/ArduinoCout.hpp>
#include <Ticker.h>
#include <Wire.h>

using arduino::cout;

// Serial config
int decimal_places = 4;
String data_seperator = "\t";

// NeoPixel
#define PIN 0
#define NUMPIXELS 1
Adafruit_NeoPixel LED(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// File system setup
#define SD_CS_PIN 14
String FILE_NAME = "/data.csv";
File myFile;
File flashFile;

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

const int dataArraySize = 25;
data_struct data[dataArraySize];
int writing_index;

struct flash_struct {
  float k_x;
  float k_y;
  float k_z;
  float raw_x;
  float raw_y;
  float raw_z;
};

flash_struct f_data[dataArraySize];

volatile bool command_active = false;

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
float BMP_start_alt, BMP_alt, last_BMP_alt, BMP_alt_vel;
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
const float IMU_FREQ = 100;
const float IMU_INTERVAL = 1 / (2 * IMU_FREQ);  // Two times because the interrupt uses flip flop
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

// Kalman filter
float dt = 1 / IMU_FREQ;
float dt_2 = pow(dt, 2) / 2;
float dt2 = pow(dt, 2);
float dt_4 = pow(dt_2, 2);
float dt_3 = pow(dt, 3) / 2;

float t = 0;

int tick;

const float acc_std = 0.35 / 1.96;
const float acc_var = pow(acc_std, 2);

const float gps_std = 3 / 1.96;
const float gps_pos_var = pow(gps_std, 2);
const float gps_vel_std = 0.1 / 1.96;
const float gps_vel_var = pow(gps_vel_std, 2);
const float bmp_std = 0.25 / 1.96;
const float bmp_pos_var = pow(bmp_std, 2);

Matrix F = {
  { 1, 0, 0, dt, 0, 0 },
  { 0, 1, 0, 0, dt, 0 },
  { 0, 0, 1, 0, 0, dt },
  { 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 1 },
};

Matrix F_T = {
  { 1, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0 },
  { dt, 0, 0, 1, 0, 0 },
  { 0, dt, 0, 0, 1, 0 },
  { 0, 0, dt, 0, 0, 1 },
};

Vector x = {
  { 0 },
  { 0 },
  { 0 },
  { 0 },
  { 0 },
  { 0 },
};

Matrix B = {
  { dt_2, 0, 0 },
  { 0, dt_2, 0 },
  { 0, 0, dt_2 },
  { dt, 0, 0 },
  { 0, dt, 0 },
  { 0, 0, dt },
};

Vector u = {
  { 0 },
  { 0 },
  { 0 },
};

Matrix P = {
  { 1, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 1 },
};

float sigma_2_dt_4 = acc_var * dt_4;
float sigma_2_dt2 = acc_var * dt2;
float sigma_2_dt_3 = acc_var * dt_3;

Matrix Q = {
  { sigma_2_dt_4, 0, 0, sigma_2_dt_3, 0, 0 },
  { 0, sigma_2_dt_4, 0, 0, sigma_2_dt_3, 0 },
  { 0, 0, sigma_2_dt_4, 0, 0, sigma_2_dt_3 },
  { sigma_2_dt_3, 0, 0, sigma_2_dt2, 0, 0 },
  { 0, sigma_2_dt_3, 0, 0, sigma_2_dt2, 0 },
  { 0, 0, sigma_2_dt_3, 0, 0, sigma_2_dt2 },
};

Vector z = {
  { 0 },
  { 0 },
  { 0 },
  { 0 },
  { 0 },
  { 0 },
};

Matrix H = {
  { 1, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 1 },
};

Matrix H_T = {
  { 1, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 1 },
};

Matrix C = {
  { 1, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0 },
  { 0, 0, 1, 0, 0 },
  { 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 1 },
  { 0, 0, 1, 0, 0 },
};

Matrix C_T = {
  { 1, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 1 },
  { 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 1, 0 },
};

Matrix Cov_z = {
  { gps_pos_var, 0, 0, 0, 0 },
  { 0, gps_pos_var, 0, 0, 0 },
  { 0, 0, bmp_pos_var, 0, 0 },
  { 0, 0, 0, gps_vel_var, 0 },
  { 0, 0, 0, 0, gps_vel_var },
};

Matrix I = {
  { 1, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 1 },
};

Vector y = {
  { 0 },
  { 0 },
  { 0 },
  { 0 },
  { 0 },
  { 0 },
};

Matrix R = C * Cov_z * C_T;

Matrix S = H * P * H_T;

Matrix K = P * H_T / S(0, 0);

// Enable components
bool ENABLE_SD = true;
bool ENABLE_Radio = false;

bool ENABLE_BMP = true;
bool ENABLE_TRM = false;
bool ENABLE_BNO = true;
bool ENABLE_GPS = true;

// Multithreading
//TaskHandle_t DHCore; // Data-Handling Core

void setup() {
  Serial.begin(2000000);

  // NeoPixel
  LED.begin();
  LED.show();

  // Status LED
  pinMode(STATUS_LED, OUTPUT);

  if (ENABLE_SD) {
    setupSD();
    setupSPIFFS();
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

  if (ENABLE_SD) {
    //deleteData();
  }

  setupBlink();

  xTaskCreatePinnedToCore(
    DHTask,
    "DataHandling",
    10000,
    NULL,
    1,
    NULL,  // &DHCore
    1);

  if (ENABLE_BNO) {
    tickerIMU.attach(IMU_INTERVAL, IMU_INTERRUPT);
  }
}

void debugMatrix(Matrix matrix) {
  cout << matrix << "\n"
       << std::endl;
}

void setupSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    errorBlink("SD");
  }
}

void setupSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    errorBlink("SPIFFS");
  }
}

void writeHeader() {
  char* header = "Seconds [s], Kalman x [m], Kalman y [m], Kalman z [m], Acceleration x [m/s^2], Acceleration y [m/s^2], Acceleration z [m/s^2], Velocity x [m/s], Velocity y [m/s], Velocity z [m/s], q0, q1, q2, q3, Latitude [degrees], Longitude [degrees], GPS Delta x [m], GPS Delta y [m], BMP Altitude [m]";
  appendFile(header);
}

void setupRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  /*
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
    while (1)
      ;
  } else {
    Serial.println("Starting LoRa successful!");
  }

  //LoRa.setSpreadingFactor(12);
  //LoRa.setSignalBandwidth(125E3);
  LoRa.setSyncWord(0xF3);
  LoRa.setTxPower(23);
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
  GPS.sendCommand("$PMTK386,0*23");
  GPS.sendCommand(PMTK_ENABLE_SBAS);
}

void setupBlink() {
  // Status LED
  digitalWrite(STATUS_LED, HIGH);

  // NeoPixel
  for (int i = 0; i < 3; i++) {
    LED.setPixelColor(0, LED.Color(255, 255, 255));
    LED.show();
    delay(50);
    LED.setPixelColor(0, LED.Color(0, 0, 0));
    LED.show();
    delay(100);
  }
}

void errorBlink(String message) {
  // Status LED
  digitalWrite(STATUS_LED, LOW);

  Serial.print("ERROR: ");
  Serial.println(message);

  delay(5000);

  // NeoPixel
  LED.setPixelColor(0, LED.Color(255, 0, 0));
  LED.show();
  delay(200);
  LED.setPixelColor(0, LED.Color(0, 0, 0));
  LED.show();
  delay(200);
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
  if (Serial.available()) {
    command_active = true;
    serialFlush();
  } else {
    return;
  }

  Serial.println();
  Serial.println("Waiting for command...");
  Serial.println();

  while (command_active) {
    if (Serial.available() > 0) {
      String receivedString = Serial.readStringUntil('\n');
      serialFlush();

      if (receivedString == "rs") {
        readSDData();
      } else if (receivedString == "rf") {
        readFlashData();
      } else if (receivedString == "ds") {
        deleteSDData();
      } else if (receivedString == "df") {
        deleteFlashData();
      } else if (receivedString == "cs") {
        checkSDData();
      } else if (receivedString == "cf") {
        checkFlashData();
      } else if (receivedString == "e") {
        command_active = false;
      } else {
        Serial.print("\t");
        Serial.println("Unknown command received: '" + receivedString + "'");
        Serial.println();
        errorBlink("Unknown command received");
      }

      if (command_active) {
        Serial.println();
        Serial.println("Waiting for command...");
      }

      Serial.println();
    }
  }
  Serial.println("Command canceled");
  Serial.println();
}

void readSDData() {
  myFile = SD.open(FILE_NAME);
  if (myFile) {
    LED.setPixelColor(0, LED.Color(0, 0, 255));
    LED.show();
    digitalWrite(STATUS_LED, LOW);
    while (myFile.available() && !Serial.available()) {
      Serial.write(myFile.read());
    }
    Serial.println();
    myFile.close();
    digitalWrite(STATUS_LED, HIGH);

    LED.setPixelColor(0, LED.Color(0, 0, 0));
    LED.show();
    delay(500);
  } else {
    Serial.print("\t");
    Serial.println("Error opening file for reading.");
    errorBlink("SD Read");
  }
}

void readFlashData() {
  myFile = SPIFFS.open(FILE_NAME);
  if (myFile) {
    LED.setPixelColor(0, LED.Color(0, 0, 255));
    LED.show();
    digitalWrite(STATUS_LED, LOW);
    while (myFile.available() && !Serial.available()) {
      Serial.write(myFile.read());
    }
    Serial.println();
    myFile.close();
    digitalWrite(STATUS_LED, HIGH);

    LED.setPixelColor(0, LED.Color(0, 0, 0));
    LED.show();
    delay(500);
  } else {
    Serial.print("\t");
    Serial.println("Error opening file for reading.");
    errorBlink("SPIFFS Read");
  }
}

void deleteSDData() {
  if (SD.exists(FILE_NAME)) {
    Serial.print("\t");
    Serial.print("Deleting file in...");
    delay(1000);
    for (int i = 3; i > 0; i--) {
      Serial.print(" " + String(i) + "...");
      LED.setPixelColor(0, LED.Color(255, 0, 0));
      LED.show();
      digitalWrite(STATUS_LED, HIGH);
      delay(500);
      LED.setPixelColor(0, LED.Color(0, 0, 0));
      LED.show();
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

void deleteFlashData() {
  if (SPIFFS.exists(FILE_NAME)) {
    Serial.print("\t");
    Serial.print("Deleting file in...");
    delay(1000);
    for (int i = 3; i > 0; i--) {
      Serial.print(" " + String(i) + "...");
      LED.setPixelColor(0, LED.Color(255, 0, 0));
      LED.show();
      digitalWrite(STATUS_LED, HIGH);
      delay(500);
      LED.setPixelColor(0, LED.Color(0, 0, 0));
      LED.show();
      digitalWrite(STATUS_LED, LOW);
      delay(500);
    }
    SPIFFS.remove(FILE_NAME);
    Serial.println(" File deleted!");
  } else {
    Serial.print("\t");
    Serial.println("[" + FILE_NAME + "] not found");
    errorBlink("SPIFFS Delete");
  }
}

void checkSDData() {
  Serial.print("\t");
  Serial.println("Used space on SD: " + String(SD.usedBytes() / (1024 * 1024)) + " / " + String(SD.totalBytes() / (1024 * 1024)) + " MB (6 MB = EMPTY)");
}

void checkFlashData() {
  Serial.print("\t");
  Serial.println("Used space on flash: " + String(SPIFFS.usedBytes() / (1024)) + " / " + String(SPIFFS.totalBytes() / (1024)) + " KB");
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

void rotateVectorWithQuaternion(float* vector, float* quaternion) {
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

void predict() {
  x = F * x + B * u;
  P = F * P * F_T + Q;
}

void update() {
  y = z - H * x;
  S = H * P * H_T + R;
  K = P * H_T / S(0, 0);
  x = x + K * y;
  P = (I - K * H) * P;
}

float randomNormal(float std_dev) {
  float u1 = random(1, 32767) / 32767.0;  // Uniform random number between 0 and 1
  float u2 = random(1, 32767) / 32767.0;  // Uniform random number between 0 and 1

  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);  // Box-Muller transform
  return std_dev * z0;                                   // Scale and shift the result
}

void IMU_INTERRUPT() {
  if (command_active) { return; }

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

          u = {
            { vector[0] },
            { vector[1] },
            { vector[2] },
          };

          predict();

          Serial.print(x(0, 0), 4);
          Serial.print(",");
          Serial.print(x(1, 0), 4);
          Serial.print(",");
          Serial.print(x(2, 0), 4);
          Serial.print(",");
          Serial.print(GPS_delta_meter_x, 4);
          Serial.print(",");
          Serial.print(GPS_delta_meter_y, 4);
          Serial.print(",");
          Serial.println(BMP_alt, 4);

          /*
          Serial.print(GPS_delta_meter_x, 4);
          Serial.print("\t");
          Serial.print(GPS_delta_meter_y, 4);
          Serial.print("\t");
          
          Serial.print(BMP_alt, 4);
          Serial.print("\t");
          Serial.print(x(0, 0), 4);
          Serial.print("\t");
          Serial.print(x(1, 0), 4);
          Serial.print("\t");
          Serial.print(x(2, 0), 4);
          Serial.print("\t");
          Serial.print(x(3, 0), 4);
          Serial.print("\t");
          Serial.print(x(4, 0), 4);
          Serial.print("\t");
          Serial.println(x(5, 0), 4);
          */

          /*
          //  Visualize Accelerometer Vector with LED
          x_color_factor = 1 / (1 + (pow(vector[0], 2) / 50));
          y_color_factor = 1 / (1 + (pow(vector[1], 2) / 50));
          z_color_factor = 1 / (1 + (pow(vector[2], 2) / 50));

          LED.setPixelColor(0, LED.Color(255 * (1 - x_color_factor), 255 * (1 - y_color_factor), 255 * (1 - z_color_factor)));
          LED.show();
          */

          if (ENABLE_SD) {
            data[writing_index].sec = float(micros()) * 0.000001;
            data[writing_index].delta_millis = delta_micros * 0.001;
            data[writing_index].kalman_x = x(0, 0);
            data[writing_index].kalman_y = x(1, 0);
            data[writing_index].kalman_z = x(2, 0);
            data[writing_index].acc_x = vector[0];
            data[writing_index].acc_y = vector[1];
            data[writing_index].acc_z = vector[2];
            data[writing_index].vel_x = x(3, 0);
            data[writing_index].vel_y = x(4, 0);
            data[writing_index].vel_z = x(5, 0);
            data[writing_index].q0 = q0;
            data[writing_index].q1 = q1;
            data[writing_index].q2 = q2;
            data[writing_index].q3 = q3;
            data[writing_index].GPS_lat = GPS_lat;
            data[writing_index].GPS_lon = GPS_lon;
            data[writing_index].GPS_m_x = GPS_delta_meter_x;
            data[writing_index].GPS_m_y = GPS_delta_meter_y;
            data[writing_index].BMP_alt = BMP_alt;

            /*
            f_data[writing_index].k_x = x(0, 0);
            f_data[writing_index].k_y = x(1, 0);
            f_data[writing_index].k_z = x(2, 0);
            f_data[writing_index].raw_x = GPS_delta_meter_x;
            f_data[writing_index].raw_y = GPS_delta_meter_y;
            f_data[writing_index].raw_z = BMP_alt;
            */

            //Serial.println(writing_index);

            writing_index++;
          }

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
  if (command_active) { return; }

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  } else {
    return;
  }

  if (GPS_ori_lat == 0 && GPS_ori_lon == 0) {
    GPS_ori_lat = GPS_lat;
    GPS_ori_lon = GPS_lon;
  }

  GPS_lat = convertCords(GPS.latitude);
  GPS_lon = convertCords(GPS.longitude);

  GPS_speed = GPS.speed * 0.51444444444444;
  GPS_angle = GPS.angle;

  GPS_vel_x = GPS_speed * cos(GPS_angle * PI / 180);
  GPS_vel_y = GPS_speed * sin(GPS_angle * PI / 180);

  DeltaMetersOutput delta_meters_output = calculateDeltaMeter(GPS_lat, GPS_lon, GPS_ori_lat, GPS_ori_lon, BMP_alt);

  GPS_delta_meter_x = -1 * delta_meters_output.x;
  GPS_delta_meter_y = -1 * delta_meters_output.y;

  if (abs(GPS_delta_meter_x) > 100000) {
    GPS_delta_meter_x = 0;
    GPS_delta_meter_y = 0;
  }

  last_millis = millis();

  if (ENABLE_BMP) {
    BMP_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    BMP_alt -= kalman_offset_z;
    BMP_alt_vel = (BMP_alt - last_BMP_alt) * BMP_freq;
    last_BMP_alt = BMP_alt;
  }

  z = {
    { GPS_delta_meter_x },
    { GPS_delta_meter_y },
    { BMP_alt },
    { GPS_vel_x },
    { GPS_vel_y },
    { BMP_alt_vel },
  };

  update();
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

  String data = String(radio_packet_num) + "," + String(sqrt(pow(x(0, 0), 2) + pow(x(1, 0), 2) + pow((x(2, 0) - BMP_start_alt), 2)));

  bool packetSent = LoRa.beginPacket() && LoRa.print(data) && LoRa.endPacket(true);

  if (packetSent != 1) { setupRadio(); }

  radio_packet_num++;

  /*
  String data = "Temp: " + String(temperature) + "  Alt: " + String(BMP_alt) + " Lat: " + String(GPS_lat, 6) + " Lon: " + String(GPS_lon, 6);
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

void DHTask(void* parameter) {
  for (;;) {
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
        //Serial.print("Radio running on core");
        //Serial.println(xPortGetCoreID());

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
        //Serial.print("Temp: ");
        //Serial.println(temperature);
      }
    }
  }
}

void loop() {
  if (writing_index >= dataArraySize - 10) {
    if (ENABLE_SD) {
      int readRange = writing_index;

      myFile = SD.open(FILE_NAME, FILE_APPEND);
      //flashFile = SPIFFS.open(FILE_NAME, FILE_APPEND);

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

          /*
          sprintf(data_char, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                  f_data[0].k_x,
                  f_data[0].k_y,
                  f_data[0].k_z,
                  f_data[0].raw_x,
                  f_data[0].raw_y,
                  f_data[0].raw_z);

          flashFile.println(data_char);
          flashFile.flush();
          */

          for (int k = 0; k < dataArraySize - 1; k++) {
            data[k] = data[k + 1];
          }
          writing_index--;
        }
        myFile.close();
        //flashFile.close();
        //Serial.println("Write complete");
        //Serial.println();
      } else {
        Serial.println("Write failed");
        Serial.println();
      }
    }
  }
}