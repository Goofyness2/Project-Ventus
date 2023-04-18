/*

VSCode arduino activation:
settings.json (add):

"arduino.path": "",
    "arduino.commandPath": "",
    "arduino.additionalUrls": [
        "https://raw.githubusercontent.com/VSChina/azureiotdevkit_tools/master/package_azureboard_index.json",
        "http://arduino.esp8266.com/stable/package_esp8266com_index.json"
    ],
    "arduino.logLevel": "info",
    "C_Cpp.intelliSenseEngine": "Tag Parser"



To do list (software):
  - Radio
    - Find right frequencies
    - Send processed data at 5 Hz (interrupt clock?)
  - GPS
    - Make sure that parsed data can be saved
  - BMP390
    - Data interrupt
  - BNO085
    - Integrate. Compare between accelerometers.
      - (Combine MPU-6050 acceleration and BNO orientation and integrate. Subtracting gravitation needed)
        - Data interrupt
    - Data interrupt
  - Fix safety system when power drops (interrupt)
  - Safety system for when sensors return NULL, the weight should be zero
  - Sensor fusion

Completed (software):
  - Radio
    - ...
  - GPS
    - ...
  - BMP390
    - Kalman filtering, 50 Hz
  - BNO085
    - Checked goofy data, not goofy anymore. Position drifts 20m/2min at rest; 20m/~3s when moved.

To do list (hardware):
  - New 3D-design for printing
  - Solve BMP390 temperature vs. pressure tradeoff issue
  - Solder
  - Find a radio antenna?
  - Check fall velocity (parachute size)
  - Safety circuit (capacitor)

Completed (hardware):
  - ...

*/

#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Ticker.h>
#include <math.h>

SoftwareSerial mySerial(2, 3);  // TX, RX
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false

const float GPS_INTERVAL = 0.2;  // 5 Hz interval
Ticker tickerBMP;
const float KALMAN_INTERVAL = 0.02;  // 50 Hz interval
Ticker tickerKALMAN;

// Kalman filter parameters
float v = 0;                 // Initial velocity
float gps_x_res_var = 1;     // Initial residual variance for the x-axis
float gps_y_res_var = 1;     // Initial residual variance for the y-axis
float dt = KALMAN_INTERVAL;  // Time step
float R_GPS = 6.25;             // Measurement noise variance
float Q_GPS = 1;             // Process noise variance
float w, res, z;

float lat1, lon1, lat2, lon2, unf_delta_x, unf_delta_y, delta_x, delta_y, delta_x_w, delta_y_w;

bool calibrated = false;

struct KalmanOutput {
  float x;
  float w;
};

struct DeltaMetersOutput {
  float x;
  float y;
};

const float circumf_ekv = 40007863;
const float circumf_avg = 40075017;

float i;

#define BUTTON_PIN 12

void setup() {
  Serial.begin(115200);
  delay(1000);

  tickerBMP.attach(GPS_INTERVAL, GPS_Interrupt);
  tickerKALMAN.attach(KALMAN_INTERVAL, KALMAN_Interrupt);

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

float convertCords(float degrees_minutes) {
  float degrees = floor(degrees_minutes / 100);
  float minutes = degrees_minutes - (degrees * 100);
  float decimal_degrees = degrees + (minutes / 60);

  return decimal_degrees;
}

DeltaMetersOutput calculateDeltaMeter(float lat1, float lon1, float lat2, float lon2) {
  float delta_lat = lat2 - lat1;
  float delta_lon = lon2 - lon1;

  float average_lat = (lat1 + lat2) / 2;

  float delta_meters_x = (delta_lon / 360) * cos(radians(average_lat)) * circumf_ekv;
  float delta_meters_y = (delta_lat / 360) * circumf_avg;

  return { delta_meters_x, delta_meters_y };
}

KalmanOutput kalmanFilter(float z, float dt, float R, float Q, float &x, float &v, float &res, float &res_var) {
  // Prediction step
  float x_pred = x + v * dt;

  // Update step
  float res_pred = z - x_pred;                      // Predicted residual
  float K = res_var / (res_var + R);                // Kalman gain
  x = x_pred + K * res_pred;                        // Updated state estimate
  v = 0;                                            // Updated velocity estimate v_pred + K * (res_pred / dt)
  res = z - x;                                      // Updated residual
  res_var = (1 - K) * res_var + K * Q * res * res;  // Updated residual variance
  float w = 1 / (res_var + R);                      // Weight of the measurement

  return { x, w };
}

void KALMAN_Interrupt() {
  KalmanOutput delta_x_output = kalmanFilter(unf_delta_x, KALMAN_INTERVAL, R_GPS, Q_GPS, delta_x, v, res, gps_x_res_var);
  delta_x = delta_x_output.x;
  delta_x_w = delta_x_output.w;

  KalmanOutput delta_y_output = kalmanFilter(unf_delta_y, KALMAN_INTERVAL, R_GPS, Q_GPS, delta_y, v, res, gps_y_res_var);
  delta_y = delta_y_output.x;
  delta_y_w = delta_y_output.w;

  Serial.print("FIX: ");
  Serial.println(GPS.fix);
  Serial.print("Latitude: ");
  Serial.println(lat2);
  Serial.print("Longitude: ");
  Serial.println(lon2);
  Serial.println();
  Serial.print("Unfiltered delta X: ");
  Serial.println(unf_delta_x);
  Serial.print("Unfiltered delta Y: ");
  Serial.println(unf_delta_y);
  Serial.println();
  Serial.print("Filtered delta X: ");
  Serial.print(delta_x);
  Serial.print(" (");
  Serial.print(delta_x_w / 0.16);
  Serial.println(")");
  Serial.print("Filtered delta Y: ");
  Serial.print(delta_y);
  Serial.print(" (");
  Serial.print(delta_y_w / 0.16);
  Serial.println(")");
  Serial.println();
  Serial.print("Distance: ");
  Serial.println(sqrt(pow(unf_delta_x, 2.0) + pow(unf_delta_y, 2.0)));
  Serial.println();
}

void GPS_Interrupt() {
  if (GPS.fix) {
    i++;
    lat1 = lat2;
    lon1 = lon2;

    lat2 = convertCords(GPS.latitude);
    lon2 = convertCords(GPS.longitude);

    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Resetting delta meters");
      unf_delta_x = 0;
      unf_delta_y = 0;
      delta_x = 0;
      delta_y = 0;
    } else if (calibrated) {
      DeltaMetersOutput delta_meters_output = calculateDeltaMeter(lat1, lon1, lat2, lon2);
      unf_delta_x += delta_meters_output.x;
      unf_delta_y += delta_meters_output.y;
    } else {
      calibrated = true;
    }
  }
}

void loop() {
  char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
}