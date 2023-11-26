#include <SPI.h>
#include <LoRa.h>

// Radio setup
#define RFM95_CS 15
#define RFM95_RST 5
#define RFM95_INT 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.69

int recieved_packet_num;
int num_lost_packets;

void setup() {
  Serial.begin(115200);
  Serial.println("LoRa Reciever");
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  if (!LoRa.begin(434.69E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  } else {
    Serial.println("Starting LoRa successful!");
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.println("Received packet:");
    Serial.println();

    // read packet
    String packetNum = "";
    String packetData = "";
    bool commaEncountered = false;

    while (LoRa.available()) {
      char packetChar = (char)LoRa.read();

      if (packetChar == ',') {
        commaEncountered = true;
        continue;
      }

      if (commaEncountered) {
        packetData += packetChar;
      } else {
        packetNum += packetChar;
      }
    }

    int packetIndex = packetNum.toInt();

    Serial.print("\tPacket num: ");
    Serial.println(packetIndex);
    Serial.print("\tData: ");
    Serial.println(packetData);
    Serial.print("\tRSSI: ");
    Serial.println(LoRa.packetRssi());
    Serial.println();
    Serial.print("\tPackets lost: ");
    Serial.println(num_lost_packets);

    if (packetIndex != recieved_packet_num + 1) {
      Serial.println();
      Serial.println("\t\t\t###\tPACKET LOSS\t##");
      num_lost_packets += packetIndex - (recieved_packet_num + 1);
    }

    recieved_packet_num = packetIndex;

    Serial.println();
  }
}
