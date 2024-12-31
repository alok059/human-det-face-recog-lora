#include <SPI.h>
#include <RH_RF95.h>

// LoRa Module Pins
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Create LoRa Driver Instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// LoRa Frequency
#define RF95_FREQ 433.0

// Timeout settings
unsigned long lastReceiveTime = 0;  // Stores the last time data was received
const unsigned long timeoutDuration = 10000;  // 10 seconds timeout

void setup() {
  Serial.begin(9600);  // Start serial communication

  // Initialize LoRa Module Pins
  pinMode(RFM95_RST, OUTPUT);

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
    incomingData.trim();  // Trim any extra whitespace or newline characters

    // Store the received data and update the last receive time
    if (incomingData.length() > 0) {
      lastReceiveTime = millis();  // Update the last receive time

      // Send the received data over LoRa
      const char* msg = incomingData.c_str();  // Convert the String to a C-string
      rf95.send((uint8_t*)msg, strlen(msg));  // Send the message over LoRa
      rf95.waitPacketSent();  // Wait for the packet to be sent before proceeding

      // Print to the Serial Monitor
      Serial.print("Received from Raspberry Pi: ");
      Serial.println(incomingData);

      Serial.print("Message sent: ");
      Serial.println(msg);
    }
  }

  // Check for timeout
  if (millis() - lastReceiveTime > timeoutDuration) {
    // Stop sending if no new data received within the timeout period
    Serial.println("No new data received. Pausing LoRa transmission.");
    delay(1000);  // Delay to prevent flooding Serial Monitor
    return;  // Exit loop iteration
  }

  delay(5000);  // Keep a delay to regulate sending rate if needed
}

//
//#include <SPI.h>
//#include <RH_RF95.h>
//
//// LoRa Module Pins
//#define RFM95_CS 10
//#define RFM95_RST 9
//#define RFM95_INT 2
//
//// Create LoRa Driver Instance
//RH_RF95 rf95(RFM95_CS, RFM95_INT);
//
//// LoRa Frequency
//#define RF95_FREQ 433.0
//
//// Timeout settings
//unsigned long lastReceiveTime = 0;  // Stores the last time data was received
//const unsigned long timeoutDuration = 10000;  // 10 seconds timeout
//
//void setup() {
//  Serial.begin(9600);  // Start serial communication
//
//  // Initialize LoRa Module Pins
//  pinMode(RFM95_RST, OUTPUT);
//
//  // Reset LoRa Module
//  digitalWrite(RFM95_RST, HIGH);
//  delay(10);
//  digitalWrite(RFM95_RST, LOW);
//  delay(10);
//  digitalWrite(RFM95_RST, HIGH);
//
//  // Initialize LoRa Module
//  if (!rf95.init()) {
//    Serial.println("LoRa init failed");
//    while (1);  // Halt execution on failure
//  }
//
//  // Configure LoRa Settings
//  rf95.setFrequency(RF95_FREQ);
//  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
//
//  Serial.println("LoRa transmitter ready");
//}
//
//void loop() {
//  // Check if new data is available from the Serial
//  if (Serial.available() > 0) {
//    String incomingData = Serial.readStringUntil('\n');  // Read the incoming serial data
//    incomingData.trim();  // Trim any extra whitespace or newline characters
//
//    // Store the received data and update the last receive time
//    if (incomingData.length() > 0) {
//      lastReceiveTime = millis();  // Update the last receive time
//
//      // Send the received data over LoRa
//      const char* msg = incomingData.c_str();  // Convert the String to a C-string
//      rf95.send((uint8_t*)msg, strlen(msg));  // Send the message over LoRa
//      rf95.waitPacketSent();  // Wait for the packet to be sent before proceeding
//
//      // Print to the Serial Monitor
//      Serial.print("Received from Raspberry Pi: ");
//      Serial.println(incomingData);
//
//      Serial.print("Message sent: ");
//      Serial.println(msg);
//    }
//  }
//
//  // Check for timeout
//  if (millis() - lastReceiveTime > timeoutDuration) {
//    // Stop sending if no new data received within the timeout period
//    Serial.println("No new data received. Pausing LoRa transmission.");
//    delay(1000);  // Delay to prevent flooding Serial Monitor
//    return;  // Exit loop iteration
//  }
//
//  delay(5000);  // Keep a delay to regulate sending rate if needed
//}
