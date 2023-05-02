void setup() {
  Serial.begin(115200);
}

void loop() {
  // Generate a sine wave with random noise
  float t = millis() / 2000.0;
  float signal = 10 * sin(t) + random(10) - 5;
  
  // Send the signal value to Serial Monitor
  Serial.println(signal);
  
  // Wait for a short time before generating the next signal
  delay(50);
}
