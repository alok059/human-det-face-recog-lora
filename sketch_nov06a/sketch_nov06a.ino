#include <SPI.h>
#include <RH_RF95.h>

// LoRa Module Pins
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define TX_LED 4  // Optional: LED for transmission status

// Create LoRa Driver Instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// LoRa Frequency
#define RF95_FREQ 433.0

// Timeout settings
unsigned long lastReceiveTime = 0;  // Last time data was received
const unsigned long timeoutDuration = 10000;  // 10-second timeout
String message = "No data yet";  // Default message

void setup() {
  Serial.begin(9600);  // Start serial communication

  // Initialize pins
  pinMode(RFM95_RST, OUTPUT);
  pinMode(TX_LED, OUTPUT);

  // Reset LoRa Module
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);

  // Initialize LoRa Module
  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);  // Halt execution on failure
  }

  // Configure LoRa Settings
  rf95.setFrequency(RF95_FREQ);
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);

  Serial.println("LoRa transmitter ready");
}

void loop() {
  // Check if new data is available from the Serial
  if (Serial.available() > 0) {
    String incomingData = Serial.readStringUntil('\n');  // Read the incoming serial data
    incomingData.trim();  // Trim whitespace and newlines

    if (incomingData.length() > 0) {
      message = incomingData;  // Update message to send
      lastReceiveTime = millis();  // Reset timeout timer

      Serial.print("Received from Raspberry Pi: ");
      Serial.println(message);
    }
  }

  // Send message over LoRa continuously
  digitalWrite(TX_LED, HIGH);  // Turn on LED to indicate transmission
  rf95.send((uint8_t*)message.c_str(), message.length());
  rf95.waitPacketSent();  // Ensure message is sent before proceeding
  digitalWrite(TX_LED, LOW);  // Turn off LED after transmission

  Serial.print("Message sent: ");
  Serial.println(message);

  // Check for timeout
  if (millis() - lastReceiveTime > timeoutDuration) {
    Serial.println("No new data received. Using default message.");
    message = "No new data";  // Reset to default message after timeout
    delay(1000);
  }

  delay(2000);  // Wait before next transmission to avoid flooding
}
