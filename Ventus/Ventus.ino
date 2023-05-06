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

#include <Arduino.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RH_RF69.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Ticker.h>
#include <Wire.h>
#include <SPI.h>


struct dataStruct {
  int millis;
  float x;
  float y;
  float z;
};


#define RF69_FREQ 868.0


#define RFM69_CS 0
#define RFM69_IRQ 15  // G0
#define RFM69_RST 16


// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_IRQ);


int16_t packetnum = 0;  // packet counter, we increment per xmission


const float IMU_INTERVAL = 0.04;  // 25 Hz interval
Ticker tickerIMU;
const float GPS_INTERVAL = 0.2;  // 5 Hz interval
Ticker tickerGPS;


SoftwareSerial mySerial(2, 13);  // TX, RX
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false


struct DeltaMetersOutput {
  float x;
  float y;
};


float lat1, lon1, lat2, lon2;


const float circumf_ekv = 40007863;
const float circumf_avg = 40075017;


bool calibrated;


#define SEALEVELPRESSURE (1023)


Adafruit_BMP3XX bmp;


// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1


Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


Adafruit_MPU6050 mpu;


const float B_x = 0.12;
const float B_y = -2.915;
const float B_z = -3.71;


const float A_x = 1.003064351;
const float A_y = 0.99949109418;
const float A_z = 0.9859437751;


float IMU_acc_x;
float IMU_acc_y;
float IMU_acc_z;


float IMU_vel_x;
float IMU_vel_y;
float IMU_vel_z;


// Kalman Filter parameters


float IMU_gamma_factor = 0.05;
float IMU_res_var = 1;        // Initial residual variance
float IMU_x = 0;              // Initial position
float IMU_dt = IMU_INTERVAL;  // Time step
float IMU_R = 0.0625;         // Measurement noise variance
float IMU_Q = 1;              // Process noise variance
float IMU_w, IMU_res, IMU_z;


float GPS_gamma_factor = 0.05;
float GPS_res_var = 1;        // Initial residual variance
float GPS_x = 0;              // Initial position
float GPS_v = 0;              // Initial velocity
float GPS_dt = GPS_INTERVAL;  // Time step
float GPS_R = 1;              // Measurement noise variance
float GPS_Q = 1;              // Process noise variance
float GPS_w, GPS_res, GPS_lat_z, GPS_lon_z, GPS_alt_z, GPS_lat_x, GPS_lon_x, GPS_alt_x, GPS_lat_v, GPS_lon_v, GPS_alt_v;


struct KalmanOutput {
  float x;
  float v;
  float w;
};


void setup(void) {
  Serial.begin(115200);


  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);


  Serial.println("Feather RFM69 TX Test!");
  Serial.println();


  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);


  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1)
      ;
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }


  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW


  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x00, 0x04, 0x00, 0x02, 0x00, 0x08, 0x01, 0x04,
                    0x00, 0x04, 0x02, 0x09, 0x00, 0x07, 0x03, 0x01 };
  rf69.setEncryptionKey(key);


  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");


  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  mySerial.println(PMTK_Q_RELEASE);


  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }


  setReports();


  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }


  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");


  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);


  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  Serial.println("Adafruit BNO08x test!");


  tickerIMU.attach(IMU_INTERVAL, IMU_Interrupt);
  tickerGPS.attach(GPS_INTERVAL, GPS_Interrupt);
}


// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable vector");
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
  vector[0] = -result[0];
  vector[1] = result[1];
  vector[2] = result[2] - 9.82;
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


  float delta_x = (delta_lon / 360) * cos(radians(average_lat)) * circumf_ekv;
  float delta_y = (delta_lat / 360) * circumf_avg;


  return { delta_x, delta_y };
}


KalmanOutput KalmanFilter(float z, float dt, float R, float Q, float gamma_factor, float &x, float &v, float &res, float &res_var) {
  // Prediction step
  float x_pred = x + v * dt;


  // Update step
  float res_pred = z - x_pred;                                                        // Predicted residual
  float K = res_var / (res_var + R);                                                  // Kalman gain
  x = x_pred + K * res_pred;                                                          // Updated state estimate
  res = z - x;                                                                        // Updated residual
  float omega = (res * res * gamma_factor) / ((res * res * gamma_factor) + R);        // Update omega
  v = (1 - omega) * v + omega * K * (res / dt);                                       // Updated velocity estimate v_pred + K * (res / dt)
  res_var = (1 - K) * res_var + K * Q * res * res;                                    // Updated residual variance
  float w = 1 / (res_var + R);                                                        // Weight of the measurement


  return { x, v, w };
}


void IMU_Interrupt() {
  if (bno08x.wasReset()) {
    Serial.println("sensor was reset ");
    setReports();
    return;
  }


  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  switch (sensorValue.sensorId) {
    case SH2_ROTATION_VECTOR:
      float micros1 = micros();


      if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
      }


      float bmp_pres = bmp.pressure;


      IMU_z = 44330.0 * (1.0 - pow(bmp_pres / (SEALEVELPRESSURE * 100), 0.1903));


      float q0 = sensorValue.un.rotationVector.real;
      float q1 = sensorValue.un.rotationVector.i;
      float q2 = sensorValue.un.rotationVector.j;
      float q3 = sensorValue.un.rotationVector.k;
      float quaternion[4] = { q0, q1, -q2, -q3 };


      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);


      float acc_x = a.acceleration.x;
      float acc_y = a.acceleration.y;
      float acc_z = a.acceleration.z;


      float cal_acc_x = A_x * (acc_x - B_x);
      float cal_acc_y = A_y * (acc_y - B_y);
      float cal_acc_z = A_z * (acc_z - B_z);


      float vector[3] = { cal_acc_y, cal_acc_x, cal_acc_z };  // Define the input vector


      rotateVectorWithQuaternion(vector, quaternion);  // Rotate the vector using the quaternion


      IMU_acc_x = vector[0];
      IMU_acc_y = vector[1];
      IMU_acc_z = vector[2];


      IMU_vel_x += IMU_acc_x * IMU_dt;
      IMU_vel_y += IMU_acc_y * IMU_dt;
      IMU_vel_z += IMU_acc_z * IMU_dt;


      KalmanOutput IMU_output = KalmanFilter(IMU_z, IMU_dt, IMU_R, IMU_Q, IMU_gamma_factor, IMU_x, IMU_vel_z, IMU_res, IMU_res_var);
      IMU_w = IMU_output.w;


      KalmanOutput GPS_z_output = KalmanFilter(GPS_alt_z, GPS_dt, GPS_R, GPS_Q, GPS_gamma_factor, GPS_alt_x, IMU_vel_z, GPS_res, GPS_res_var);
      GPS_w = GPS_z_output.w;


      KalmanOutput GPS_x_output = KalmanFilter(GPS_lat_z, GPS_dt, GPS_R, GPS_Q, GPS_gamma_factor, GPS_lat_x, IMU_vel_x, GPS_res, GPS_res_var);


      KalmanOutput GPS_y_output = KalmanFilter(GPS_lon_z, GPS_dt, GPS_R, GPS_Q, GPS_gamma_factor, GPS_lon_x, IMU_vel_y, GPS_res, GPS_res_var);


      float micros2 = micros();


      float deltaMicros = micros2 - micros1;


      /*


      Serial.println();
      Serial.println();
      Serial.println();


      Serial.print("Quaternion - r: ");
      Serial.print(q0);
      Serial.print(" i: ");
      Serial.print(q1);
      Serial.print(" j: ");
      Serial.print(q2);
      Serial.print(" k: ");
      Serial.println(q3);


      // Print the raw vector
      Serial.print("Raw vector - X: ");
      Serial.print(-cal_acc_y);
      Serial.print("   Y: ");
      Serial.print(cal_acc_x);
      Serial.print("   Z: ");
      Serial.println(cal_acc_z);


      // Print the rotated vector
      Serial.print("Rotated vector - X: ");
      Serial.print(IMU_acc_x);
      Serial.print("   Y: ");
      Serial.print(IMU_acc_y);
      Serial.print("   Z: ");
      Serial.println(IMU_acc_z);


      Serial.println();


      //Print the Kalman filter output
      Serial.print("Kalman Filter state: ");
      Serial.print(IMU_x);
      Serial.print(" m   velocity: ");
      Serial.print(IMU_v);
      Serial.print(" m/s   weight: ");
      Serial.println(IMU_w);


      // Print the our own altitude equation for BMP
      Serial.print("Raw altitude from BMP: ");
      Serial.print(IMU_z);
      Serial.println(" m");


      Serial.println();


      // Print FIX
      Serial.print("FIX: ");
      Serial.println(GPS.fix);


      // Print satelite count
      Serial.print("Satellites: ");
      Serial.println(GPS.satellites);


      // Print GPS coordinates
      Serial.print("GPS delta meter - Latitude: ");
      Serial.print(GPS_lat_z);
      Serial.print(" m   Longitude: ");
      Serial.print(GPS_lon_z);
      Serial.print(" m   Altitude: ");
      Serial.println(GPS_alt_z);
      Serial.print("GPS Kalman Filter - Latitude: ");
      Serial.print(GPS_lat_x);
      Serial.print(" m   Longitude: ");
      Serial.print(GPS_lon_x);
      Serial.print(" m   Altitude: ");
      Serial.println(GPS_alt_x);


      // Print BMP data
      Serial.print("BMP: ");
      Serial.println(IMU_x);


      */


      // Print the rotated vector
      Serial.print("Rotated vector - X: ");
      Serial.print(IMU_acc_x);
      Serial.print("   Y: ");
      Serial.print(IMU_acc_y);
      Serial.print("   Z: ");
      Serial.println(IMU_acc_z);


      // Print GPS coordinates
      Serial.print("GPS delta meter - Latitude: ");
      Serial.print(GPS_lat_z);
      Serial.print(" m   Longitude: ");
      Serial.print(GPS_lon_z);
      Serial.print(" m   Altitude: ");
      Serial.println(GPS_alt_z);


      Serial.print("Kalman Filter output - X: ");
      Serial.print(GPS_lat_x);
      Serial.print(" m   Y: ");
      Serial.print(GPS_lon_x);
      Serial.print(" m   Z: ");
      Serial.println(IMU_x);


      // Print weight
      Serial.print("GPS weight: ");
      Serial.println(GPS_w);
      Serial.print("BMP weight: ");
      Serial.println(IMU_w);


      Serial.println();


      // Print the delta tima
      Serial.print("Delta time: ");
      Serial.print(deltaMicros);
      Serial.println(" microseconds");


      break;
  }
}


void GPS_Interrupt() {
  if (GPS.fix) {
    lat1 = lat2;
    lon1 = lon2;


    lat2 = convertCords(GPS.latitude);
    lon2 = convertCords(GPS.longitude);
    GPS_alt_z = convertCords(GPS.altitude);


    if (!calibrated) {
      GPS_lat_z = 0;
      GPS_lon_z = 0;
      calibrated = true;
    } else {
      DeltaMetersOutput delta_meters_output = calculateDeltaMeter(lat1, lon1, lat2, lon2);
      GPS_lon_z += delta_meters_output.x;
      GPS_lat_z += delta_meters_output.y;
    }
  }
 
  String sensorData = String(GPS_lat_x) + "," + String(GPS_lon_x) + "," + String(IMU_x); // + ",   Latitude: " + String(convertCords(GPS.latitude)) + " Longitude: " + String(convertCords(GPS.longitude)) + " Altitude: " + String(convertCords(GPS.altitude))


  char payload[128];
  sensorData.toCharArray(payload, 128);
  Serial.print("Sending ");
  Serial.println(payload);
  rf69.send((uint8_t *)payload, strlen(payload));
}


void loop() {
  // Es ist leer
  char c = GPS.read();


  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
}

