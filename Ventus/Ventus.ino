void setup() {
  Serial.begin(115200);
}

void loop() {
  // Generate a sine wave with random noise
  float t = millis() / 2000.0;

  float x = 10 * sin(t) + random(10) - 5;
  float y = 5 * cos(t) + random(3) - 3;
  float z = 4 * sin(t / 3) + random(4) - 4;

  Serial.print(x);  Serial.print(",");
  Serial.print(y);  Serial.print(",");
  Serial.println(z);
  
  // Wait for a short time before generating the next signal
  delay(20);
}
