void setup() {
  Serial.begin(115200);
}

float generateGaussianNoise(float mu, float sigma) {
  static bool generate;
  static float z0, z1;
  generate = !generate;

  if (!generate)
     return z1 * sigma + mu;

  float u1, u2;
  do {
     u1 = random(10000) * (1.0 / 10000.0);
     u2 = random(10000) * (1.0 / 10000.0);
  } while (u1 <= 1e-7);

  float mag = sqrt(-2.0 * log(u1));
  z0 = mag * cos(6.283185307179586476925286766559 * u2);
  z1 = mag * sin(6.283185307179586476925286766559 * u2);
  return z0 * sigma + mu;
}

void loop() {
  // Generate a sine wave with random noise
  float t = millis() / 2000.0;

  float x = generateGaussianNoise(10 * sin(t), 0.2);
  float y = generateGaussianNoise(5 * cos(t), 0.2);
  float z = generateGaussianNoise(4 * sin(t / 3), 0.2);

  Serial.print(x);  Serial.print(",");
  Serial.print(y);  Serial.print(",");
  Serial.println(z);
  
  // Wait for a short time before generating the next signal
  delay(100);
}
