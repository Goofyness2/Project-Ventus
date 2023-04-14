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
    - Kalman filtering
    - 5 Hz or 50 Hz?
  - BNO085
    - Check goofy data
    - Integrate. Compare between accelerometers.
      - (Combine MPU-6050 acceleration and BNO orientation and integrate. Subtracting gravitation needed)
  - Fix safety system when power drops (interrupt)
  - Safety system for when sensors return NULL, the weight should be zero
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
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

struct euler_t2 {
  float yaw;
  float pitch;
  float roll;
} ypr2;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float acc_x, acc_y, acc_z;
float mag_x, mag_y, mag_z;

void setReports() {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Adafruit BNO085 data!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO085 Found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEuler2(float qr, float qi, float qj, float qk, euler_t2* ypr2, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr2->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr2->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr2->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr2->yaw *= RAD_TO_DEG;
      ypr2->pitch *= RAD_TO_DEG;
      ypr2->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    //Serial.println(micros());    
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_ROTATION_VECTOR:
        quaternionToEuler2(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, &ypr2, true);
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mag_x = sensorValue.un.magneticField.x;
        mag_y = sensorValue.un.magneticField.y;
        mag_z = sensorValue.un.magneticField.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        acc_x = sensorValue.un.linearAcceleration.x;
        acc_y = sensorValue.un.linearAcceleration.y;
        acc_z = sensorValue.un.linearAcceleration.z;
        break;
    }

    //Serial.println(sensorValue.status);
    Serial.print(ypr.yaw);  Serial.print(", ");
    Serial.print(ypr.pitch);  Serial.print(", ");
    Serial.println(ypr.roll);
    Serial.print(ypr2.yaw);  Serial.print(", ");
    Serial.print(ypr2.pitch);  Serial.print(", ");
    Serial.println(ypr2.roll);
    Serial.println("");
  }

}
