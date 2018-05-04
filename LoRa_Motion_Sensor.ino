// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>


#define MOTION_PIN 0
#define LED1_PIN 12
#define LED2_PIN 11

/* for feather32u4 */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7


/* for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
*/

/* for shield 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 7
*/

/* Feather 32u4 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Feather m0 w/wing 
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"
*/

#if defined(ESP8266)
  /* for ESP w/featherwing */ 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"

#elif defined(ESP32)  
  /* ESP32 feather w/wing */
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A

#elif defined(NRF52)  
  /* nRF52832 feather w/wing */
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// If set, flashes until the millis() time
unsigned long flash_saw_motion;
unsigned long flash_received_motion;

int16_t packetnum = 0;  // packet counter, we increment per xmission


void motion_detected()
{
  Serial.println("Movement Detected");
  flash_saw_motion = millis() + 5 * 1000;
}

void tx_motion() {
  char radiopacket[20] = "Motion #           ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
}


void setup() 
{
  flash_saw_motion = 0;
  flash_received_motion = 0;
  
  // Motion Sensor Interrupts
  attachInterrupt( digitalPinToInterrupt( MOTION_PIN ), motion_detected, RISING );
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(100);

  Serial.println("---=== Feather LoRa Motion Detector  ===---");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void blink(int led) {
    Serial.println("blink!");
    digitalWrite( led, HIGH );
    delay(100);
    digitalWrite( led, LOW );
    delay(100);
}

void loop() {

  // Blink the LED if flash is set
  if ( flash_saw_motion > millis() )  {
    blink(LED1_PIN);
    tx_motion();
  }
  if ( flash_received_motion > millis() )  {
    blink(LED2_PIN);
  }

  // Check for received messages
  if ( rf95.available() ) {
    // Receive Buffer
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);
  
    if ( rf95.recv(buf, &buflen) ) {
      Serial.print( "Received Message: " );
      Serial.println( (char*)buf );

      if ( flash_received_motion < millis() ) { // If we're not already flashing
        flash_received_motion = millis() + 10 * 1000; // flash for 10 sec
      }
    } else {
      Serial.println( "Receive Failed" );
    }
  }

}


