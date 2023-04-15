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

void setup() {
    // ...
    Serial.begin(115200);
}


void loop() {
    // ...
    Serial.print("HEJJJJJ");
    delay(100);
}
