
/*
   * Author: Andreas Spiess, 2017
   * 
  This sketch measures the time to send a ESP-Now message. It is a strip down of Anthony's sensor sketch
https://github.com/HarringayMakerSpace/ESP-Now
Anthony Elder
*/
#include "ESP8266WiFi.h"



  
  int device = 11;
  int temperature = 76;
  int humidity = 49;
  int pressure = 95;
  
extern "C" void preinit() {

   // Change MAC 

 //uint8_t mac[] = { 0xb4, 0xe6, 0x52, 0x44, 0x86, 0xad };
    uint8_t mac[6];
    mac[0] = temperature;
    mac[1] = humidity;
    mac[2] = pressure;
    mac[3] = pressure;
    mac[4] = random(256);
    mac[5] = random(256);
  
  wifi_set_opmode (STATION_MODE);
  wifi_set_macaddr(STATION_IF, mac);
}

extern "C" {
#include <espnow.h>
}

    

// this is the MAC Address of the remote ESP server which receives these sensor readings
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

#define WIFI_CHANNEL 1
//#define SLEEP_SECS 15 * 60 // 15 minutes
#define SLEEP_SECS 5000  
#define SEND_TIMEOUT 245  // 245 millis seconds timeout 

#define MESSAGELEN 10

unsigned long entry;

// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA {
  int temperature = temperature;
  int humidity = humidity;
  int pressure = pressure;
  char testdata[MESSAGELEN];
} sensorData;


volatile boolean callbackCalled;

unsigned long entry1 = millis();





    
void prinScanResult(int networksFound)
{
  Serial.printf("%d network(s) found\n", networksFound);
  for (int i = 0; i < networksFound; i++)
  {
    Serial.printf("%d: %s, Ch:%d (%ddBm) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
  }
}


void setup()
{
  int i = 0;
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP_Now SENDER");
  
 // WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(10);
  WiFi.scanNetworksAsync(prinScanResult);
  
  WiFi.hostname("Livingroom");
  Serial.println(WiFi.macAddress());
  Serial.printf("target mac: %02x%02x%02x%02x%02x%02x", remoteMac[0], remoteMac[1], remoteMac[2], remoteMac[3], remoteMac[4], remoteMac[5]); 
  Serial.printf(", channel: %i\n", WIFI_CHANNEL); 

   if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    //gotoSleep();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  Serial.println(millis() - entry1);
  unsigned long entry2 = millis();
  esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
  Serial.println(millis() - entry2);
  unsigned long entry3 = millis();

  esp_now_register_send_cb([](uint8_t* mac, uint8_t sendStatus) {
    Serial.printf("send_cb, send done, status = %i\n", sendStatus);
    callbackCalled = true;
  });
  Serial.println(millis() - entry3);
  unsigned long entry4 = millis();

  callbackCalled = false;

  for (i = 0; i < MESSAGELEN; i++) sensorData.testdata[i] = '0';
  sensorData.testdata[MESSAGELEN] = '\0';
  
  }


 void loop() {



 WiFi.scanNetworksAsync(prinScanResult);
 Serial.println(WiFi.macAddress());
 Serial.println(WiFi.hostname());
 delay(10);
  uint8_t bs[sizeof(sensorData)];
  memcpy(bs, &sensorData, sizeof(sensorData));
  unsigned long entry = millis();
  esp_now_send(NULL, bs, sizeof(sensorData)); // NULL means send to all peers
  Serial.print("Time to send: ");
  Serial.println(millis() - entry);
  Serial.print("Overall Time: ");
  Serial.println(millis() - entry1);
  Serial.print("Size: ");
  Serial.println(sizeof(bs));
  //ESP.deepSleep(0);
 delay(10000);
 
  }

void gotoSleep() {                            //need connection between GPIO16 and reset pin on ESP8266
  // add some randomness to avoid collisions with multiple devices
  int sleepSecs = SLEEP_SECS;// + ((uint8_t)RANDOM_REG32/2);
  Serial.printf("Up for %i ms, going to sleep for %i secs...\n", millis(), sleepSecs);
  ESP.deepSleep(sleepSecs * 1000000, RF_NO_CAL);
}
