/*

To do list (software):
  - Radio
    - Find right frequencies
    - Send processed data at 5 Hz (interrupt clock?)
  - GPS
    - Make sure that parsed data can be saved
  - BMP390
    - Kalman filtering
    - 5 Hz or 50 Hz?
  - BNO085
    - Check goofy data
    - Integrate. Compare between accelerometers.
      - (Combine MPU-6050 acceleration and BNO orientation and integrate. Subtracting gravitation needed)
  - Fix safety system when power drops (interrupt)
  - Sensor fusion

Completed (software):
  - Radio
    - ...
  - GPS
    - ...
  - BMP390
    - ...
  - BNO085
    - ...

To do list (hardware):
  - New 3D-design for printing
  - Solve BMP390 temperature vs. pressure tradeoff issue
  - Solder
  - Find a radio antenna?
  - Check fall velocity (parachute size)
  - Safety circuit (capacitor)

Completed (hardware):
  -

*/

#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Ticker.h>
#include <math.h>

#define SEALEVELPRESSURE (102614)

Adafruit_BMP3XX bmp;

float BMPalt, BMPw, BMPalt1, BMPalt2, delta_BMP;

// Kalman filter parameters
float x = 0;              // Initial position
float v = 0;              // Initial velocity
float res_var = 1;        // Initial residual variance
float R = 2.5;            // Measurement noise variance
float Q = 1;              // Process noise variance
float dt, w, res, z;      // Remaining parameters

struct KalmanOutput {
  float x;
  float w;
};

const float BMP_INTERVAL = 0.02;  // 50 Hz interval
Ticker tickerBMP;

const float GPS_INTERVAL = 0.2;  // 5 Hz interval
Ticker tickerGPS;

SoftwareSerial mySerial(2, 3);  // TX, RX
Adafruit_GPS GPS(&mySerial);

float lat1, lat2, lon1, lon2, delta_x, delta_y, delta_z, unf_delta_x, unf_delta_y, delta_x_w, delta_y_w, GPSw;

float earth_radius = 6371000.0;

#define GPSECHO false

#define BUTTON_PIN 12

void setup() {
  Serial.begin(115200);
  delay(1000);

  tickerBMP.attach(BMP_INTERVAL, BMP_Interrupt);
  tickerGPS.attach(GPS_INTERVAL, GPS_Interrupt);

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // 9600 or 4800 baud
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

  delay(200);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void BMP_Interrupt() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  KalmanOutput output = kalmanFilter(bmp.readAltitude(SEALEVELPRESSURE), BMP_INTERVAL, R, Q, x, v, res, res_var);
  BMPalt = output.x;
  BMPw = output.w;
}

float convertCords(float degrees_minutes) {
  float degrees = floor(degrees_minutes / 100);
  float minutes = degrees_minutes - (degrees * 100);
  float decimal_degrees = degrees + (minutes / 60);

  return decimal_degrees;
}

KalmanOutput kalmanFilter(float z, float dt, float R, float Q, float &x, float &v, float &res, float &res_var) {
  // Prediction step
  float x_pred = x + v * dt;
  float v_pred = v;

  // Update step
  float res_pred = z - x_pred;          // Predicted residual
  float K = res_var / (res_var + R);    // Kalman gain
  x = x_pred + K * res_pred;            // Updated state estimate
  v = 0;     // Updated velocity estimate v_pred + K * (res_pred / dt)
  res = z - x;                          // Updated residual
  res_var = (1 - K) * res_var + K * Q;  // Updated residual variance
  float w = 1 / (res_var + R);          // Weight of the measurement

  return { x, w };
}

void GPS_Interrupt() {
  Serial.print("Fix: ");
  Serial.println(GPS.fix);
  Serial.println(" ");

  if (GPS.fix) {
    lat2 = convertCords(GPS.latitude);
    lon2 = convertCords(GPS.longitude);

    float lat1_rad = radians(lat1);
    float lon1_rad = radians(lon1);
    float lat2_rad = radians(lat2);
    float lon2_rad = radians(lon2);

    float delta_lat = lat2_rad - lat1_rad;
    float delta_lon = lon2_rad - lon1_rad;

    unf_delta_x += earth_radius * delta_lon * cos(lat1_rad);
    unf_delta_y += earth_radius * delta_lat;

    BMPalt2 = BMPalt;
    
    delta_BMP = BMPalt2 - BMPalt1;
    delta_z += delta_BMP;

    KalmanOutput output = kalmanFilter(unf_delta_x, GPS_INTERVAL, R, Q, x, v, res, res_var);
    delta_x = output.x;
    delta_x_w = output.w;

    KalmanOutput output = kalmanFilter(unf_delta_y, GPS_INTERVAL, R, Q, x, v, res, res_var);
    delta_y = output.x;
    delta_y_w = output.w;

    GPSw = (delta_x_w + delta_y_w) / 2;

    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Resetting delta meters");
      unf_delta_x = 0;
      unf_delta_y = 0;
      delta_x = 0;
      delta_y = 0;
      delta_z = 0;
    }

    lat1 = lat2;
    lon1 = lon2;
    BMPalt1 = BMPalt2;

    Serial.print("X: ");
    Serial.print(delta_x);
    Serial.println(" ");
    Serial.print("Y: ");
    Serial.print(delta_y);
    Serial.println(" ");
    Serial.print("Z: ");
    Serial.print(delta_z);
    Serial.print(" (");
    Serial.print(delta_BMP);
    Serial.print(")");
    Serial.println(" ");
    Serial.print("BMP weight: ");
    Serial.print(BMPw);
    Serial.println(" ");
    Serial.print("GPS weight: ");
    Serial.print(GPSw);
    Serial.println(" ");
    Serial.print("Diagonal: ");
    Serial.println(sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0)));
    Serial.println(" ");
    Serial.print("Latitude: ");
    Serial.print(lat2);
    Serial.println(" ");
    Serial.print("Longitude: ");
    Serial.print(lon2);
    Serial.println(" ");
    Serial.print("Altitude: ");
    Serial.println(alt2);
    Serial.println(" ");
    Serial.print("Speed: ");
    Serial.print(GPS.speed * 1.852);
    Serial.println(" km/h");
    Serial.print("Satellites: ");
    Serial.println((int)GPS.satellites);
    Serial.println(" ");
  }
}

void loop() {  // run over and over again
  char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
}