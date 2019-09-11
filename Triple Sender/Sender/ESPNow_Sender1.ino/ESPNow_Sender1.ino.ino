
// EspnowController.ino
// Oroginal code is here : https://forum.arduino.cc/index.php?topic=551310.0 
// a minimal program derived from
//          https://github.com/HarringayMakerSpace/ESP-Now

// This is the program that sends the data. (The Controller)

//=============
ADC_MODE(ADC_VCC); //vcc read-mode
//#define VCC_ADJ 1.096
#include <ESP8266WiFi.h>
#include "SparkFunBME280.h"

  int device = 01;
  int temperature = 86;
  int humidity = 49;
  int pressure = 96;
  int battery = 29;
  int lux = 67;
  
   
   extern "C" void preinit() {
    
   // Change MAC 

 //uint8_t mac[] = { 0xb4, 0xe6, 0x52, 0x44, 0x86, 0xad };
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



extern "C" {
    #include <espnow.h>
}

    // this is the MAC Address of the slave which receives the data
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

#define WIFI_CHANNEL 4
#define SLEEP_SECS 5000 
#define SEND_TIMEOUT 245  // 245 millis seconds timeout 

    // must match the slave struct
struct __attribute__((packed)) DataStruct {
    char text[32];
    float temperature;
    float humidity;
    float pressure;
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

BME280 bme280;

byte ledPin = 2;

void prinScanResult(int networksFound)
{
  Serial.printf("%d network(s) found\n", networksFound);
  for (int i = 0; i < networksFound; i++)
  {
    Serial.printf("%d: %s, Ch:%d (%ddBm) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
  }
}


//==============

void setup() {
    Serial.begin(115200); Serial.println();
    Serial.println("Starting EspnowController.ino");
    battery = ((int (ESP.getVcc()) / 95.454)); 
  // read sensor first before awake generates heat
    readBME280();
    //WiFi.mode(WIFI_STA); // Station mode for esp-now controller
    WiFi.disconnect();
    delay(10);
    WiFi.scanNetworksAsync(prinScanResult);

    WiFi.hostname("Livingroom");
    Serial.println(WiFi.macAddress());
    Serial.printf("target mac: %02x%02x%02x%02x%02x%02x", remoteMac[0], remoteMac[1], remoteMac[2], remoteMac[3], remoteMac[4], remoteMac[5]); 
    Serial.printf(", channel: %i\n", WIFI_CHANNEL); 
    
    Serial.printf("slave mac: %02x%02x%02x%02x%02x%02x", remoteMac[0], remoteMac[1], remoteMac[2], remoteMac[3], remoteMac[4], remoteMac[5]);
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

    strcpy(myData.text, "ESPNow Data Received : ");
    Serial.print("Message "); Serial.println(myData.text);

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);

    Serial.println("Setup finished");

}

//==============

void loop() {
 //battery = ((int (ESP.getVcc()) / 95.454)); 
 
 WiFi.scanNetworksAsync(prinScanResult);
 Serial.println(battery);
 Serial.println(WiFi.macAddress());
 Serial.println(WiFi.hostname());
 delay(10);
    readBME280();
    sendData();
    blinkLed();
  //ESP.deepSleep(0);
    delay(10);
}

//==============

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

//==============

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

//================

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
  Serial.print(temperature);
  Serial.print(humidity);
  Serial.print(pressure);
  myData.temperature = temperature;
  myData.humidity = humidity;
  myData.pressure = pressure;
  Serial.printf("temperature=%01f, humidity=%01f, pressure=%01f\n", myData.temperature, myData.humidity, myData.pressure);
}



 void gotoSleep() {                            //need connection between GPIO16 and reset pin on ESP8266
  // add some randomness to avoid collisions with multiple devices
  int sleepSecs = SLEEP_SECS;// + ((uint8_t)RANDOM_REG32/2);
  Serial.printf("Up for %i ms, going to sleep for %i secs...\n", millis(), sleepSecs);
  ESP.deepSleep(sleepSecs * 1000000, RF_NO_CAL);
}
