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

// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"

Adafruit_SGP30 sgp;

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 868.0

#define RFM69_CS      0
#define RFM69_IRQ     15   // G0
#define RFM69_RST     16
// #define LED           0

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_IRQ);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  // pinMode(LED, OUTPUT);     
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
    while (1);
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
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
}



void loop() {
  delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

  String radiopacket;

  if (!sgp.IAQmeasure() || !sgp.IAQmeasureRaw()) {
    radiopacket = "Measurement failed!";
  } else {
    radiopacket = "TVOC: " + String(sgp.TVOC) + " ppb/t, eCO2: " + String(sgp.eCO2) + " ppm, Raw H2: " + String(sgp.rawH2) + " /t, Raw Ethanol: " + String(sgp.rawEthanol);
  }

  Serial.print("Sending "); Serial.println(radiopacket);
    
  char hej[5] = "hej";
  
  // Send a message!
  rf69.send((uint8_t *)hej, strlen(hej));
  rf69.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      // Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
