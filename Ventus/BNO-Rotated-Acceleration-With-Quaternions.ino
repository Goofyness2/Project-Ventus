#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <Ticker.h>
#include <Wire.h>

int16_t packetnum = 0;  // packet counter, we increment per xmission

// For SPI mode, we need a CS pin
#define BNO08X_CS 27
#define BNO08X_INT 15
#define BNO08X_RESET 14

#define PIN 0
#define NUMPIXELS 1

Adafruit_NeoPixel LED(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float acc_x, acc_y, acc_z, q0, q1, q2, q3, x_color_factor, y_color_factor, z_color_factor;

bool FlipFlop;

unsigned long first_micros, second_micros, last_micros;

int decimal_places = 4;
const float IMU_INTERVAL = 0.005;  // 400 Hz interval
Ticker tickerIMU;

void setup() {
  Serial.begin(115200);

  LED.begin();
  LED.show();

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Couldn't initialize BNO085!");
  }
  setReports();
  tickerIMU.attach(IMU_INTERVAL, IMU_INTERRUPT);
}

void setReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("Could not enable linear acceleration");
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

void IMU_INTERRUPT() {
  first_micros = micros();
  if (bno08x.wasReset()) {
    setReports();
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

        /*  Visualize Accelerometer Vector with LED
        x_color_factor = 1 / (1 + (pow(vector[0], 2) / 50));
        y_color_factor = 1 / (1 + (pow(vector[1], 2) / 50));
        z_color_factor = 1 / (1 + (pow(vector[2], 2) / 50));

        LED.setPixelColor(0, LED.Color(255 * (1 - x_color_factor), 255 * (1 - y_color_factor), 255 * (1 - z_color_factor)));
        LED.show();
        */

        float delta_micros = second_micros - first_micros;
        Serial.print(micros() - last_micros);
        last_micros = micros();

        Serial.print("\t");
        Serial.print(vector[0], decimal_places);
        Serial.print("\t");
        Serial.print(vector[1], decimal_places);
        Serial.print("\t");
        Serial.print(vector[2], decimal_places);
        Serial.print("\t");
        Serial.print(q0, decimal_places);
        Serial.print("\t");
        Serial.print(q1, decimal_places);
        Serial.print("\t");
        Serial.print(q2, decimal_places);
        Serial.print("\t");
        Serial.println(q3, decimal_places);

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

void loop() {
}