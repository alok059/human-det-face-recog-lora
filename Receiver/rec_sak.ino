#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);
int rxLED = 4;
int buzzer = 3;

#define RF95_FREQ 433.0

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(rxLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  while (!Serial);

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }

  rf95.setFrequency(RF95_FREQ);
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);

  Serial.println("LoRa receiver ready");
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      buf[len] = '\0'; // Null-terminate the received data
      String receivedData = String((char*)buf);

      Serial.print("Received: ");
      Serial.println(receivedData);

      // Indicate message received with LED
      digitalWrite(rxLED, HIGH);
      delay(100); 
      digitalWrite(rxLED, LOW);

      // Check the received message and take appropriate action
      if (receivedData == "A") {
        Serial.println("Safe");
        digitalWrite(buzzer,HIGH);
        delay(50);
        digitalWrite(buzzer,LOW);
        delay(10);
        digitalWrite(buzzer,HIGH);
        delay(50);
        digitalWrite(buzzer,LOW);
        
      } else if (receivedData == "B") {
        Serial.println("Enemy");
        digitalWrite(buzzer, HIGH);
        delay(1000); // Buzzer on for 5 seconds
        digitalWrite(buzzer, LOW);
      }
      else if (receivedData == "C") {
        Serial.println("OtherObjects");
        digitalWrite(rxLED, HIGH);
        delay(1000); 
        digitalWrite(rxLED, LOW);
      }

    } else {
      Serial.println("Receive failed");
    }
  }
}