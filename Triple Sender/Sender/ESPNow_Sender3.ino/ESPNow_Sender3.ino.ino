
// EspnowController.ino
// Oroginal code is here : https://forum.arduino.cc/index.php?topic=551310.0 
// a minimal program derived from
//          https://github.com/HarringayMakerSpace/ESP-Now

// This is the program that sends the data. (The Controller)

/* IRremoteESP8266: IRsendDemo - demonstrates sending IR codes with IRsend.
 *
 * Version 1.1 January, 2019
 * Based on Ken Shirriff's IrsendDemo Version 0.1 July, 2009,
 * Copyright 2009 Ken Shirriff, http://arcfn.com
 *
 * An IR LED circuit *MUST* be connected to the ESP8266 on a pin
 * as specified by kIrLed below.
 *
 * TL;DR: The IR LED needs to be driven by a transistor for a good result.
 *
 * Suggested circuit:
 *     https://github.com/crankyoldgit/IRremoteESP8266/wiki#ir-sending
 *
 * Common mistakes & tips:
 *   * Don't just connect the IR LED directly to the pin, it won't
 *     have enough current to drive the IR LED effectively.
 *   * Make sure you have the IR LED polarity correct.
 *     See: https://learn.sparkfun.com/tutorials/polarity/diode-and-led-polarity
 *   * Typical digital camera/phones can be used to see if the IR LED is flashed.
 *     Replace the IR LED with a normal LED if you don't have a digital camera
 *     when debugging.
 *   * Avoid using the following pins unless you really know what you are doing:
 *     * Pin 0/D3: Can interfere with the boot/program mode & support circuits.
 *     * Pin 1/TX/TXD0: Any serial transmissions from the ESP8266 will interfere.
 *     * Pin 3/RX/RXD0: Any serial transmissions to the ESP8266 will interfere.
 *   * ESP-01 modules are tricky. We suggest you use a module with more GPIOs
 *     for your first time. e.g. ESP-12 etc.
 */

//=======================================================================


#include <ESP8266WiFi.h>
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "SparkFunBME280.h"

//==============================ESPNow defines starts=======================
extern "C" {
    #include <espnow.h>
}


ADC_MODE(ADC_VCC); //vcc read-mode

// this is the MAC Address of the remote ESP server which receives these sensor readings
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

#define WIFI_CHANNEL 4
#define SLEEP_SECS 15 * 60 // 15 minutes
#define SEND_TIMEOUT 245  // 245 millis seconds timeout 
#define VOLT_LIMIT 3.00
byte ledPin = 2;

BME280 bme280;


 // must match the slave struct
struct __attribute__((packed)) DataStruct {
  char text[32];
  float device;
  float temperature;
  float humidity;
  float pressure;
  float battery;
  float lux;
    
  unsigned long time;
    
};

DataStruct myData;

unsigned long lastSentMillis;
unsigned long sendIntervalMillis = 1000;
unsigned long sentMicros;
unsigned long ackMicros;

unsigned long lastBlinkMillis;
unsigned long fastBlinkMillis = 200;
unsigned long slowBlinkMillis = 700;
unsigned long blinkIntervalMillis = slowBlinkMillis;
  int device = 01;
  int temperature = 86;
  int humidity = 49;
  int pressure = 96;
  int battery;
  int lux = 67; 
 
//==============================ESPNow defines ends=======================  


//==============================Probe Request defines starts=======================  
extern "C" void preinit() {
    
   // Change MAC 

    uint8_t mac[6];
    mac[0] = temperature;
    mac[1] = humidity;
    mac[2] = pressure;
    mac[3] = battery;
    mac[4] = lux;
    mac[5] = device;
  
  wifi_set_opmode (STATION_MODE);
  wifi_set_macaddr(STATION_IF, mac);
}
//==============================Probe Request defines ends=======================  


//====================IR defines starts============================

const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.

// Example Samsung A/C state captured from IRrecvDumpV2.ino
uint8_t samsungState[kSamsungAcStateLength] = {temperature, humidity, pressure, battery, lux, device}; //{0xb4, 0xe6, 0x52, 0x44, 0x86, 0xad, 0xb4, 0xe6, 0x52, 0x44, 0x86, 0xad };
//uint64_t hwaddr = 0x1234567890AB;//{temperature, humidity, pressure, battery, lux, device};  // 48bits 



//====================IR defines ends==============================



   

//==============================Probe Request defines=======================  





  



//======================================================================

void setup() {
    Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
    Serial.println();
    
    irsend.begin();
    readBME280();
    WiFi.mode(WIFI_STA); // Station mode for esp-now controller
    WiFi.disconnect();
    delay(10);
    WiFi.scanNetworksAsync(prinScanResult);

    float Voltage = ESP.getVcc() / (float)1023; // * (float)1.07;
    Serial.print("Voltage: "); Serial.print(Voltage); Serial.print("  Voltage Expected: "); Serial.println(VOLT_LIMIT);
    if (Voltage < VOLT_LIMIT)      // if voltage of battery gets to low, the LED wil blink fast.
  {
    Serial.println("Warning :- Battery Voltage low please change batteries" );
  }
    WiFi.hostname("Livingroom");
    
    Serial.println(WiFi.macAddress());
    Serial.printf("Target mac: %02x%02x%02x%02x%02x%02x", remoteMac[0], remoteMac[1], remoteMac[2], remoteMac[3], remoteMac[4], remoteMac[5]); 
    Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());
    Serial.printf(", channel: %i\n", WIFI_CHANNEL);
   
       if (esp_now_init() != 0) {
        Serial.println("*** ESP_Now init failed");
        while(true) {};
        //gotoSleep();
       
    }
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);

    esp_now_register_send_cb(sendCallBackFunction);

    strcpy(myData.text, "ESPNow Data:- ");
    Serial.print(WiFi.hostname()); Serial.println(myData.text);

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);

    Serial.println("Setup finished");
    Serial.println("Starting ESPNow network sender");
    Serial.println("Starting Probe request sender");
    Serial.println("Starting IR data sender");
}




//========================Main Loop================================

void loop() {

//===========================IR loop===============================

  Serial.println("Sending sensor data to Master Node");
  irsend.sendSamsungAC(samsungState);
   //irsend.sendMidea(hwaddr);
  for(uint64_t u=0; u<6; u++)
   {
      Serial.print(" Device :  ");
      Serial.print(samsungState[5]);
      Serial.print(" Temprature :  ");
      Serial.print(samsungState[0]);
      Serial.print(" Humidity :  ");
      Serial.print(samsungState[1]);
      Serial.print(" Pressure :  ");
      Serial.print(samsungState[2]);
      Serial.print(" Battery :  ");
      Serial.print(samsungState[3]);
      Serial.print(" Lux :  ");
      Serial.println(samsungState[4]);
      //Serial.println(hwaddr[u]);
   }
      
      
      

 //=====================Probe request Loop=========================
 
 WiFi.scanNetworksAsync(prinScanResult);
 Serial.print("Device MAC ID: ");
 Serial.println(WiFi.macAddress());
 Serial.print("Device Name: ");
 Serial.println(WiFi.hostname());
 delay(10);

 //============================ESPNow loop=========================
    readBME280();
    sendData();
    blinkLed();
  //ESP.deepSleep(0);
    delay(10);
}

//=========================Main Loop ends==========================


//=========================ESPNow functions starts=================

void sendData() {
    if (millis() - lastSentMillis >= sendIntervalMillis) {
        lastSentMillis += sendIntervalMillis;
        myData.time = millis();
        uint8_t bs[sizeof(myData)];
        memcpy(bs, &myData, sizeof(myData));
        sentMicros = micros();
        esp_now_send(NULL, bs, sizeof(myData)); // NULL means send to all peers
        Serial.println("sent data");
    }
}



void sendCallBackFunction(uint8_t* mac, uint8_t sendStatus) {
    ackMicros = micros();
    Serial.print("Trip micros "); Serial.println(ackMicros - sentMicros);
    Serial.printf("Send status = %i", sendStatus);
    Serial.println();
    Serial.println();
    if (sendStatus == 0) {
        blinkIntervalMillis = fastBlinkMillis;
    }
    else {
        blinkIntervalMillis = slowBlinkMillis;
    }
}



void blinkLed() {
    if (millis() - lastBlinkMillis >= blinkIntervalMillis) {
        lastBlinkMillis += blinkIntervalMillis;
        digitalWrite(ledPin, ! digitalRead(ledPin));
    }
}


void readBME280() {
  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = 0x76;
  bme280.settings.runMode = 2; // Forced mode with deepSleep
  bme280.settings.tempOverSample = 1;
  bme280.settings.pressOverSample = 1;
  bme280.settings.humidOverSample = 1;
  Serial.print("bme280 init="); Serial.println(bme280.begin(), HEX);
  temperature = bme280.readTempC();
  humidity = bme280.readFloatHumidity();
  pressure = bme280.readFloatPressure() / 100.0;
  
  myData.temperature = temperature;
  myData.humidity = humidity;
  myData.pressure = pressure;
  Serial.print("BME280 data:- ");
  Serial.printf("Temperature = %01f, Humidity = %01f, Pressure = %01f\n", myData.temperature, myData.humidity, myData.pressure);
}

void gotoSleep() {                            //need connection between GPIO16 and reset pin on ESP8266
  // add some randomness to avoid collisions with multiple devices
  int sleepSecs = SLEEP_SECS;// + ((uint8_t)RANDOM_REG32/2);
  Serial.printf("Up for %i ms, going to sleep for %i secs...\n", millis(), sleepSecs);
  ESP.deepSleep(sleepSecs * 1000000, RF_NO_CAL);
}

//=========================ESPNow functions ends===================

//=========================Probe request function starts===========
void prinScanResult(int networksFound)
{
  
  Serial.printf("%d network(s) found\n", networksFound);
  for (int i = 0; i < networksFound; i++)
  {
    Serial.printf("%d: %s, Ch:%d (%ddBm) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
  
  }
}


//=======================Probe request function ends================
 
 
 
