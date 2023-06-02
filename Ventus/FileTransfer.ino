#include <FS.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Ticker.h>
#include <math.h>

bool debug;
bool bypass;

SoftwareSerial mySerial(2, 13);  // TX, RX
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false

float lat1, lon1, lat2, lon2, alt1, alt2, delta_x, delta_y, delta_z;

struct DeltaMetersOutput {
  float x;
  float y;
};

bool calibrated = false;

const float circumf_ekv = 40007863;
const float circumf_avg = 40075017;

const float GPS_INTERVAL = 0.2;  // 50 Hz interval
Ticker tickerGPS;

void handleCommand() {
  while (debug) {
    while (!Serial.available())
      ;
    char command = Serial.read();
    Serial.println();

    if (command == 'f') {
      Serial.println("Formatting flash chip...");
      SPIFFS.format();
      Serial.println("Flash chip formatted");
    } else if (command == 'r') {
      Serial.println("Reading from flash chip:");
      File file = SPIFFS.open("/data.csv", "r");
      if (file) {
        for (int i = 0; i < 25; i++) {
          if (file.available()) {
            String line = file.readStringUntil('\n');
            Serial.println(line);
          }
        }
        file.close();
        Serial.println("File read");
      } else {
        Serial.println("File could not be opened");
      }
    } else if (command == 'w') {
      Serial.println("Writing to flash chip...");
      File file = SPIFFS.open("/data.csv", "a");
      if (file) {
        file.println("1,2,3");
        file.println("Test");
        Serial.println("Wrote to flash chip");
        file.close();
      }
    } else if (command == 'd') {
      debug = false;
      bypass = true;
    }
  }
}

void transferData() {
  Serial.println("Transferring file from flash chip...");
  File file = SPIFFS.open("/data.csv", "r");
  if (file) {
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  } else {
    Serial.println("File could not be opened");
  }
}

void setup() {
  Serial.begin(115200);

  SPIFFS.begin();

  delay(1000);

  Serial.println();
  Serial.println("Waiting for command...");

  delay(5000);

  if (Serial.available()) {
    debug = true;
    handleCommand();
  }
  Serial.println("No command available");

  if (!bypass) {
    transferData();
  }

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

  tickerGPS.attach(GPS_INTERVAL, GPS_Interrupt);
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

void GPS_Interrupt() {
  if (!GPS.begin(9600)) {
    GPS.begin(9600);

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  }
  char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
  if (GPS.fix) {

    lat1 = lat2;
    lon1 = lon2;
    alt1 = alt2;

    lat2 = convertCords(GPS.latitude);
    lon2 = convertCords(GPS.longitude);
    alt2 = GPS.altitude;

    DeltaMetersOutput delta_meters_output = calculateDeltaMeter(lat1, lon1, lat2, lon2);
    delta_x += delta_meters_output.x;
    delta_y += delta_meters_output.y;

    delta_z += alt2 - alt1;

    if (!calibrated) {
      delta_x = 0;
      delta_y = 0;
      delta_z = 0;
      calibrated = true;
    } else {
      File file = SPIFFS.open("/data.csv", "a");
      if (file) {
        file.println(String(delta_x) + "," + String(delta_y) + "," + String(delta_z));
        file.close();
      } else {
        Serial.println("Failed to open file");
      }
    }
  }

  Serial.print(delta_x);
  Serial.print(",");
  Serial.print(delta_y);
  Serial.print(",");
  Serial.print(delta_z);
  Serial.print(",");
  Serial.println(GPS.satellites);
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    debug = true;
    handleCommand();
  }

  char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
}