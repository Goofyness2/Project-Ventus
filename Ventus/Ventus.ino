void setup() {
  // Set up the serial communication at 115200 baud
  Serial.begin(115200);
}

void loop() {
  // Generate a random value between 0 and 1023
  int value = random(1024);

  // Write the value to the serial port
  Serial.println(value);

  // Wait for a short period of time
  delay(50);
}
