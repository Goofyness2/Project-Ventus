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
#include <Adafruit_Sensor.h>
#include <Ticker.h>

#define SEALEVELPRESSURE (1026.14)

Adafruit_BMP3XX bmp;

float filtered_x, filtered_v, weight;

const float BMP_INTERVAL = 0.05;  // 20 Hz interval
Ticker tickerBMP;

// Kalman filter parameters
float x = 0;              // Initial position
float v = 0;              // Initial velocity
float res_var = 1;        // Initial residual variance
float dt = BMP_INTERVAL;  // Time step
float R = 0.25;           // Measurement noise variance
float alpha = 2;          // Process noise variance
float w, res, z;

struct KalmanOutput {
  float x;
  float v;
  float w;
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  tickerBMP.attach(BMP_INTERVAL, BMP_Interrupt);

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

KalmanOutput kalmanFilter(float z, float dt, float R, float alpha, float &x, float &v, float &res, float &res_var) {
  // Prediction step
  float x_pred = x + v * dt;
  float v_pred = v;

  // Update step
  float res_pred = z - x_pred;                          // Predicted residual
  float K = res_var / (res_var + R);                    // Kalman gain
  x = x_pred + K * res_pred;                            // Updated state estimate
  v = v_pred + K * (res_pred / dt);                     // Updated velocity estimate
  res = z - x;                                          // Updated residual
  res_var = (1 - K) * res_var + K * alpha * res * res;  // Updated residual variance
  float w = 1 / (res_var + R);                          // Weight of the measurement

  return { x, v, w };
}

void BMP_Interrupt() {
  z = bmp.readAltitude(SEALEVELPRESSURE);

  KalmanOutput output = kalmanFilter(z, dt, R, alpha, x, v, res, res_var);
  filtered_x = output.x;
  filtered_v = output.v;
  weight = output.w;

  Serial.print(z);
  Serial.print(",");
  Serial.print(filtered_x);
  Serial.print(",");
  Serial.print(filtered_v);
  Serial.print(",");
  Serial.println(weight);
}

void loop() {
  // put your main code here, to run repeatedly:
}