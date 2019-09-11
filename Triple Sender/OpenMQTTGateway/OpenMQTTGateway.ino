/*
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR signal  and a MQTT broker
   Send and receiving command by MQTT

  This program enables to:
 - receive MQTT data from a topic and send signal (RF, IR, BLE, GSM)  corresponding to the received MQTT data
 - publish MQTT data to a different topic related to received signals (RF, IR, BLE, GSM)

  Copyright: (c)Florian ROBERT

    This file is part of OpenMQTTGateway.

    OpenMQTTGateway is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenMQTTGateway is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// EspnowSlave.ino
// Oroginal code is here : https://forum.arduino.cc/index.php?topic=551310.0 
// a minimal program derived from
//          https://github.com/HarringayMakerSpace/ESP-Now

// This is the program that receives the data. (The Slave)


#include "User_config.h"

// array to store previous received RFs, IRs codes and their timestamps
#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  #define array_size 12
  unsigned long ReceivedSignal[array_size][2] ={{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
  //Time used to wait for an interval before checking system measures
  unsigned long timer_sys_measures = 0;
#else // boards with smaller memory
  #define array_size 4
  unsigned long ReceivedSignal[array_size][2] ={{0,0},{0,0},{0,0},{0,0}};
#endif

//#include <PubSubClient.h>
#include <ArduinoJson.h>

// Modules config inclusion
#if defined(ZgatewayRF) || defined(ZgatewayRF2) || defined(ZgatewayPilight)
  #include "config_RF.h"
#endif
#ifdef ZgatewayRF315
  #include "config_RF315.h"
#endif
#ifdef ZgatewayLORA
  #include "config_LORA.h"
#endif
#ifdef ZgatewaySRFB
  #include "config_SRFB.h"
#endif
#ifdef ZgatewayBT
  #include "config_BT.h"
#endif
#ifdef ZgatewayIR
  #include "config_IR.h"
#endif
#ifdef Zgateway2G
  #include "config_2G.h"
#endif
#ifdef ZactuatorONOFF
  #include "config_ONOFF.h"
#endif
#ifdef ZsensorINA226
  #include "config_INA226.h"
#endif
#ifdef ZsensorHCSR501
  #include "config_HCSR501.h"
#endif
#ifdef ZsensorADC
  #include "config_ADC.h"
#endif
#ifdef ZsensorBH1750
  #include "config_BH1750.h"
#endif
#ifdef ZsensorTSL2561
  #include "config_TSL2561.h"
#endif
#ifdef ZsensorBME280
  #include "config_BME280.h"
#endif
#ifdef ZsensorDHT
  #include "config_DHT.h"
#endif
#ifdef ZgatewayRFM69
  #include "config_RFM69.h"
#endif
#ifdef ZsensorGPIOInput
  #include "config_GPIOInput.h"
#endif
#ifdef ZsensorGPIOKeyCode
  #include "config_GPIOKeyCode.h"
#endif
#ifdef ZmqttDiscovery
  #include "config_mqttDiscovery.h"
#endif
#ifdef ZactuatorFASTLED
  #include "config_FASTLED.h"
#endif

/*------------------------------------------------------------------------*/

//adding this to bypass the problem of the arduino builder issue 50
void callback(char*topic, byte* payload,unsigned int length);

bool connectedOnce = false; //indicate if we have been connected once to MQTT

int failure_number = 0; // number of failure connecting to MQTT

#ifdef ESP32
  #include <FS.h> 
  #include "SPIFFS.h"
  #include <WiFi.h>
  #include <ArduinoOTA.h>
  #include <WiFiUdp.h>
  WiFiClient eClient;
  #include <WiFiManager.h>  
  #ifdef MDNS_SD
    #include <ESPmDNS.h>
  #endif
#elif defined(ESP8266)
  #include <FS.h> 
  #include <ESP8266WiFi.h>
 // #include <ArduinoOTA.h>
 // #include <DNSServer.h>
 // #include <ESP8266WebServer.h>
 // #include <WiFiManager.h>  
 // WiFiClient eClient;
  #ifdef MDNS_SD
  //  #include <ESP8266mDNS.h>
  #endif
#else
 // #include <Ethernet.h>
 // EthernetClient eClient;
#endif
//=============================ESPNow & Proberequet defines starts=================
extern "C" {
    #include <espnow.h>
    #include <user_interface.h>
}

// it seems that the mac address needs to be set before setup() is called
//      and the inclusion of user_interface.h facilitates that
//      presumably there is a hidden call to the function initVariant()

/* Set a private Mac Address
 *  http://serverfault.com/questions/40712/what-range-of-mac-addresses-can-i-safely-use-for-my-virtual-machines
 * Note: by setting a specific MAC you can replace this slave ESP8266 device with a new one
 * and the new slave will still pick up the data from controllers which use that MAC
 */
uint8_t mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};


void initVariant() {
  WiFi.mode(WIFI_AP);
  wifi_set_macaddr(SOFTAP_IF, &mac[0]);
}



#define WIFI_CHANNEL 4

   // must match the slave struct
struct __attribute__((packed)) DataStruct {
  char text[32];
  unsigned int time;
  int device;
  int temperature;
  int humidity;
  int pressure;
  int battery;
  int light;
 };

DataStruct myData;
//================================Probe Request defines starts==========
#define CONTROLLER_ID 01

#define SWITCH 12
#define DEVICE_BUTTON 0

WiFiEventHandler probeRequestPrintHandler;
volatile boolean buttonPressed;
//================================Probe Request defines ends==========

//=============================ESPNow & Probe Request block ends=================


// client link to pubsub mqtt
//PubSubClient client(eClient);

//MQTT last attemps reconnection date
//unsigned long lastReconnectAttempt = 0;

void revert_hex_data(char * in, char * out, int l){
  //reverting array 2 by 2 to get the data in good order
  int i = l-2 , j = 0; 
  while ( i != -2 ) {
    if (i%2 == 0) out[j] = in[i+1];
    else  out[j] = in[i-1];
    j++;
    i--;
  }
  out[l-1] = '\0';
}

void extract_char(char * token_char, char * subset, int start ,int l, bool reverse, bool isNumber){
    char tmp_subset[l+1];
    memcpy( tmp_subset, &token_char[start], l );
    tmp_subset[l] = '\0';
    if (isNumber){
      char tmp_subset2[l+1];
      if (reverse) revert_hex_data(tmp_subset, tmp_subset2, l+1);
      else strncpy( tmp_subset2, tmp_subset , l+1);
      long long_value = strtoul(tmp_subset2, NULL, 16);
      sprintf(tmp_subset2, "%ld", long_value);
      strncpy( subset, tmp_subset2 , l+1);
    }else{
      if (reverse) revert_hex_data(tmp_subset, subset, l+1);
      else strncpy( subset, tmp_subset , l+1);
    }
    subset[l] = '\0';
}

int strpos(char *haystack, char *needle) //from @miere https://stackoverflow.com/users/548685/miere
{
   char *p = strstr(haystack, needle);
   if (p)
      return p - haystack;
   return -1;
}

bool to_bool(String const& s) { // thanks Chris Jester-Young from stackoverflow
     return s != "0";
}

//trace
void trc(String msg){
  #ifdef TRACE
    Serial.println(msg);
    digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
    pub(subjectTRACEtoMQTT,msg);
  #endif
}

void trc(int msg){
  #ifdef TRACE
    Serial.println(msg);
    digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
    pub(subjectTRACEtoMQTT,msg);
  #endif
}

void trc(unsigned int msg){
  #ifdef TRACE
    Serial.println(msg);
    digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
    pub(subjectTRACEtoMQTT,msg);
  #endif
}

void trc(long msg){
  #ifdef TRACE
    Serial.println(msg);
    digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
    pub(subjectTRACEtoMQTT,msg);
  #endif
}

void trc(unsigned long msg){
  #ifdef TRACE
    Serial.println(msg);
    digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
    pub(subjectTRACEtoMQTT,msg);
  #endif
}

void trc(double msg){
  #ifdef TRACE
    Serial.println(msg);
    digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
    pub(subjectTRACEtoMQTT,msg);
  #endif
}

void trc(float msg){
  #ifdef TRACE
    Serial.println(msg);
    digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
    pub(subjectTRACEtoMQTT,msg);
  #endif
}

void trc(JsonObject& data){
      char JSONmessageBuffer[JSON_MSG_BUFFER];
      data.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
      trc(JSONmessageBuffer);
}

void pub(char * topic, char * payload, bool retainFlag){
//    client.publish(topic, payload, retainFlag);
}

void pub(char * topicori, JsonObject& data){
  
    digitalWrite(led_receive, HIGH);
    
    String topic = topicori;
    #ifdef valueAsASubject
      unsigned long value = data["value"];
      if (value != 0){
        topic = topic + "/"+ String(value);
      }
    #endif
    
    #ifdef jsonPublishing
      char JSONmessageBuffer[JSON_MSG_BUFFER];
      trc(F("Pub json into:"));
      trc(topic);
      data.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
      trc(JSONmessageBuffer);
      pub(topic, JSONmessageBuffer);
    #endif

    #ifdef simplePublishing
      trc(F("Pub data per topic"));
      // Loop through all the key-value pairs in obj 
      for (JsonPair& p : data) {
        #if defined(ESP8266)
          yield();
        #endif
        if (p.value.is<unsigned long>() && strcmp(p.key, "rssi") != 0) { //test rssi , bypass solution due to the fact that a int is considered as an unsigned long
          trc(p.key);
          trc(p.value.as<unsigned long>());
          if (strcmp(p.key, "value") == 0){ // if data is a value we don't integrate the name into the topic
            pub(topic,p.value.as<unsigned long>());
          }else{ // if data is not a value we integrate the name into the topic
            pub(topic + "/" + String(p.key),p.value.as<unsigned long>());
          }
        }else if (p.value.is<int>()) {
          trc(p.key);
          trc(p.value.as<int>());
          pub(topic + "/" + String(p.key),p.value.as<int>());
        } else if (p.value.is<float>()) {
          trc(p.key);
          trc(p.value.as<float>());
          pub(topic + "/" + String(p.key),p.value.as<float>());
        } else if (p.value.is<char*>()) {
          trc(p.key);
          trc(p.value.as<const char*>());
          pub(topic + "/" + String(p.key),p.value.as<const char*>());
        }
      }
    #endif

}

void pub(char * topic, char * payload){
//    client.publish(topic, payload);
}

void pub(String topic, char *  payload){
 //   client.publish((char *)topic.c_str(),payload);
}

void pub(char * topic, unsigned long payload){
    char val[11];
    sprintf(val, "%lu", payload);
  //  client.publish(topic,val);
}

void pub(char * topic, String payload){
   // client.publish(topic,(char *)payload.c_str());
}

void pub(String topic, String payload){
   // client.publish((char *)topic.c_str(),(char *)payload.c_str());
}

void pub(String topic, int payload){
    char val[12];
    sprintf(val, "%d", payload);
   // client.publish((char *)topic.c_str(),val);
}

void pub(String topic, float payload){
    char val[12];
    dtostrf(payload,3,1,val);
  //  client.publish((char *)topic.c_str(),val);
}

void pub(char * topic, float payload){
    char val[12];
    dtostrf(payload,3,1,val);
  //  client.publish(topic,val);
}

void pub(char * topic, int payload){
    char val[6];
    sprintf(val, "%d", payload);
  //  client.publish(topic,val);
}

void pub(char * topic, unsigned int payload){
    char val[6];
    sprintf(val, "%u", payload);
  //  client.publish(topic,val);
}

void pub(char * topic, long payload){
    char val[11];
    sprintf(val, "%l", payload);
  //  client.publish(topic,val);
}

void pub(char * topic, double payload){
    char val[16];
    sprintf(val, "%d", payload);
   // client.publish(topic,val);
}

void pub(String topic, unsigned long payload){
    char val[11];
    sprintf(val, "%lu", payload);
  //  client.publish((char *)topic.c_str(),val);
}
/*
//bool reconnect() {

  // Loop until we're reconnected
  //while (!client.connected()) {
      trc(F("MQTT connection...")); //F function enable to decrease sram usage
      if (client.connect(Gateway_Name, mqtt_user, mqtt_pass, will_Topic, will_QoS, will_Retain, will_Message)) {
      trc(F("Connected to broker"));
      failure_number = 0;
      // Once connected, publish an announcement...
      pub(will_Topic,Gateway_AnnouncementMsg,will_Retain);
      // publish version
      pub(version_Topic,OMG_VERSION,will_Retain);

      //Subscribing to topic
      if (client.subscribe(subjectMQTTtoX)) {
        #ifdef ZgatewayRF
          client.subscribe(subjectMultiGTWRF); // subject on which other OMG will publish, this OMG will store these msg and by the way don't republish them if they have been already published
        #endif
        #ifdef ZgatewayRF315
          client.subscribe(subjectMultiGTWRF315);// subject on which other OMG will publish, this OMG will store these msg and by the way don't republish them if they have been already published
        #endif
        #ifdef ZgatewayIR
          client.subscribe(subjectMultiGTWIR);// subject on which other OMG will publish, this OMG will store these msg and by the way don't republish them if they have been already published
        #endif
        trc(F("Subscription OK to the subjects"));
      }
      } else {
      failure_number ++; // we count the failure
      trc(F("failure_number"));
      trc(failure_number);
      trc(F("failed, rc="));
      trc(client.state());
      trc(F("try again in 5s"));
      // Wait 5 seconds before retrying
      delay(5000);

      if (failure_number > maxMQTTretry){
        trc(F("failed connecting to mqtt"));
        return false;
      }
    }
  }
  return client.connected();
}

// Callback function, when the gateway receive an MQTT value on the topics subscribed this function is called
void callback(char* topic, byte* payload, unsigned int length) {
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.
  trc(F("Hey I got a callback "));
  // Allocate the correct amount of memory for the payload copy
  byte* p = (byte*)malloc(length + 1);
  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  // Conversion to a printable string
  p[length] = '\0';
  //launch the function to treat received data if this data concern OpenMQTTGateway
  if ((strstr(topic, subjectMultiGTWKey) != NULL) || (strstr(topic, subjectGTWSendKey) != NULL))  receivingMQTT(topic,(char *) p);
  // Free the memory
  free(p);
}
*/
void setup()
{
  //Launch serial for debugging purposes
  Serial.begin(SERIAL_BAUD);
  Serial.println();

  //================================Probe Request setup starts==========
    pinMode(SWITCH, OUTPUT);
    pinMode(DEVICE_BUTTON, INPUT);
   attachInterrupt(DEVICE_BUTTON, buttonPress, HIGH); 
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Controller", "<notused>", 6, 0, 0);
    probeRequestPrintHandler = WiFi.onSoftAPModeProbeRequestReceived(&onProbeRequest);
  
  Serial.println("Starting Probe Request receiver");  
 //================================Probe Request setup ends==========  
 
 
 //================================ESPNow setup starts==========    
    Serial.println("Starting Espnow receiver");
   
     
    Serial.print("This node AP mac: "); Serial.println(WiFi.softAPmacAddress());
    Serial.print("This node STA mac: "); Serial.println(WiFi.macAddress());
    
    
    if (esp_now_init()!=0) {
        Serial.println("*** ESP_Now init failed");
        while(true) {};
    }
    
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

    esp_now_register_recv_cb(receiveCallBackFunction);
 
    

    Serial.println("End of setup - waiting for Probe Requests and ESPNow messages");
 //================================EspNow setup ends========== 

  
  #if defined(ESP8266) || defined(ESP32)
  
    #ifdef ESP8266
      #ifndef ZgatewaySRFB // if we are not in sonoff rf bridge case we apply the ESP8266 pin optimization
        Serial.end();
        Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_TX_ONLY);// enable on ESP8266 to free some pin
      #endif
    #endif
    
    #if defined(ESPWifiManualSetup)
//      setup_wifi();
    #else
//      setup_wifimanager(false);
    #endif

    trc(F("OpenMQTTGateway mac: "));
    trc(WiFi.macAddress()); 

    trc(F("OpenMQTTGateway ip: "));
    trc(WiFi.localIP().toString());
  #endif

  //setup LED status
  pinMode(led_receive, OUTPUT);
  pinMode(led_send, OUTPUT);
  pinMode(led_info, OUTPUT);
  digitalWrite(led_receive, LOW);
  digitalWrite(led_send, LOW);
  digitalWrite(led_info, LOW);
  #ifdef ZsensorBME280
   setupZsensorBME280();
  #endif
  #ifdef ZsensorBH1750
    setupZsensorBH1750();
  #endif
  #ifdef ZsensorTSL2561
    setupZsensorTSL2561();
  #endif
  #ifdef ZactuatorONOFF
    setupONOFF();
  #endif
  #ifdef Zgateway2G
    setup2G();
  #endif
  #ifdef ZgatewayIR
    setupIR();
  #endif
  #ifdef ZgatewayLORA
    setupLORA();
  #endif
  #ifdef ZgatewayRF
    setupRF();
  #endif
  #ifdef ZgatewayRF315
    setupRF315();
  #endif
  #ifdef ZgatewayRF2
    setupRF2();
  #endif
  #ifdef ZgatewayPilight
    setupPilight();
  #endif
  #ifdef ZgatewaySRFB
    setupSRFB();
  #endif
  #ifdef ZgatewayBT
    setupBT();
  #endif
  #ifdef ZgatewayRFM69
    setupRFM69();
  #endif
  #ifdef ZsensorINA226
    setupINA226();
  #endif
  #ifdef ZsensorHCSR501
    setupHCSR501();
  #endif
  #ifdef ZsensorGPIOInput
    setupGPIOInput();
  #endif
  #ifdef ZsensorGPIOKeyCode
   setupGPIOKeyCode();
  #endif
  #ifdef ZactuatorFASTLED
    setupFASTLED();
  #endif

  }


void loop()
{
//=============================ESPNow & Probe Request block starts=================
  if (buttonPressed) {
    digitalWrite(SWITCH, !digitalRead(SWITCH));
    Serial.print("Button pressed, swiched: "); Serial.println(digitalRead(SWITCH) ? "on" : "off");
    delay(250); // to debounce button
    buttonPressed = false;
   
  }
 


//=============================ESPNow & Probe Request block ends=================
  
  digitalWrite(led_receive, LOW);
  digitalWrite(led_info, LOW);
  digitalWrite(led_send, LOW);

  unsigned long now = millis();
    #ifdef ZsensorBME280
      MeasureTempHumAndPressure(); //Addon to measure Temperature, Humidity, Pressure and Altitude with a Bosch BME280
    #endif
    #ifdef ZsensorBH1750
      MeasureLightIntensity(); //Addon to measure Light Intensity with a BH1750
    #endif
    #ifdef ZsensorTSL2561
      MeasureLightIntensityTSL2561();
    #endif
    #ifdef ZsensorDHT
      MeasureTempAndHum(); //Addon to measure the temperature with a DHT
    #endif
    #ifdef ZsensorINA226
      MeasureINA226(); //Addon to measure the temperature with a DHT
    #endif
    #ifdef ZsensorHCSR501
      MeasureHCSR501();
    #endif
    #ifdef ZsensorGPIOInput
      MeasureGPIOInput();
    #endif
    #ifdef ZsensorGPIOKeyCode
      MeasureGPIOKeyCode();
    #endif
    #ifdef ZsensorADC
      MeasureADC(); //Addon to measure the analog value of analog pin
    #endif
    #ifdef ZgatewayLORA
      LORAtoMQTT();
    #endif
    #ifdef ZgatewayRF
      RFtoMQTT();
    #endif
    #ifdef ZgatewayRF315
      RF315toMQTT();
    #endif
    #ifdef ZgatewayRF2
      RF2toMQTT();
    #endif
    #ifdef ZgatewayPilight
      PilighttoMQTT();
    #endif
    #ifdef ZgatewayBT
        #ifndef ESP32
          if(BTtoMQTT())
          trc(F("BTtoMQTT OK"));
        #endif
    #endif
    #ifdef ZgatewaySRFB
      SRFBtoMQTT();
    #endif
    #ifdef ZgatewayIR
      IRtoSERIAL();
    #endif
    #ifdef Zgateway2G
      if(_2GtoMQTT()){
      trc(F("2GtoMQTT OK"));
      }
    #endif
    #ifdef ZgatewayRFM69
      if(RFM69toMQTT())
      trc(F("RFM69toMQTT OK"));
    #endif
    #if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
   //   stateMeasures();
    #endif
    #ifdef ZactuatorFASTLED
      FASTLEDLoop();
    #endif

}

//=Loop ends here

ICACHE_RAM_ATTR void buttonPress() {
  buttonPressed = true;
}


#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
void stateMeasures(){
    unsigned long now = millis();
    if (now > (timer_sys_measures + TimeBetweenReadingSYS)) {//retriving value of memory ram every TimeBetweenReadingSYS
      timer_sys_measures = millis();
      StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
      JsonObject& SYSdata = jsonBuffer.createObject();
      trc(F("Uptime (s)"));    
      unsigned long uptime = millis()/1000;
      trc(uptime);
      SYSdata["uptime"] = uptime;
      #if defined(ESP8266) || defined(ESP32)
        uint32_t freeMem;
        freeMem = ESP.getFreeHeap();
        SYSdata["freeMem"] = freeMem;
        long rssi = WiFi.RSSI();
        SYSdata["rssi"] = rssi;
        String SSID = WiFi.SSID();
        SYSdata["SSID"] = SSID;
     //   SYSdata["ip"] = ip2CharArray(WiFi.localIP());
        String mac = WiFi.macAddress();
     //   SYSdata["mac"] = (char*)mac.c_str();
      #else
     //   SYSdata["ip"] = ip2CharArray(Ethernet.localIP());
      #endif
     String modules = "";
      #ifdef ZgatewayRF
          modules = modules + ZgatewayRF;
      #endif
      #ifdef ZgatewayRF315
          modules = modules + ZgatewayRF315;
      #endif
      #ifdef ZsensorBME280
          modules = modules + ZsensorBME280;
      #endif
      #ifdef ZsensorBH1750
          modules = modules + ZsensorBH1750;
      #endif
      #ifdef ZsensorTSL2561
          modules = modules + ZsensorTSL2561;
      #endif
      #ifdef ZactuatorONOFF
          modules = modules + ZactuatorONOFF;
      #endif
      #ifdef Zgateway2G
          modules = modules + Zgateway2G;
      #endif
      #ifdef ZgatewayIR
          modules = modules + ZgatewayIR;
      #endif
      #ifdef ZgatewayLORA
          modules = modules + ZgatewayLORA;
      #endif
      #ifdef ZgatewayRF2
          modules = modules + ZgatewayRF2;
      #endif
      #ifdef ZgatewayPilight
          modules = modules  + ZgatewayPilight;
      #endif
      #ifdef ZgatewaySRFB
          modules = modules + ZgatewaySRFB;
      #endif
      #ifdef ZgatewayBT
          modules = modules + ZgatewayBT;
      #endif
      #ifdef ZgatewayRFM69
          modules = modules + ZgatewayRFM69;
      #endif
      #ifdef ZsensorINA226
          modules = modules + ZsensorINA226;
      #endif
      #ifdef ZsensorHCSR501
          modules = modules + ZsensorHCSR501;
      #endif
      #ifdef ZsensorGPIOInput
          modules = modules + ZsensorGPIOInput;
      #endif
      #ifdef ZsensorGPIOKeyCode
          modules = modules + ZsensorGPIOKeyCode;
      #endif
      #ifdef ZsensorGPIOKeyCode
          modules = modules  + ZsensorGPIOKeyCode;
      #endif
      #ifdef ZmqttDiscovery
          modules = modules  + ZmqttDiscovery;
          pubMqttDiscovery();
      #endif
      #ifdef ZactuatorFASTLED
          modules = modules + ZactuatorFASTLED;
      #endif

      SYSdata["modules"] = modules;
      trc(SYSdata);
      char JSONmessageBuffer[JSON_MSG_BUFFER];
      SYSdata.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
      pub(subjectSYStoMQTT,JSONmessageBuffer);
      
    }
}

#endif


void storeValue(unsigned long MQTTvalue){
    unsigned long now = millis();
    // find oldest value of the buffer
    int o = getMin();
    trc(F("Min ind: "));
    trc(o);
    // replace it by the new one
    ReceivedSignal[o][0] = MQTTvalue;
    ReceivedSignal[o][1] = now;
    trc(F("store code :"));
    trc(String(ReceivedSignal[o][0])+"/"+String(ReceivedSignal[o][1]));
    trc(F("Col: val/timestamp"));
    for (int i = 0; i < array_size; i++)
    {
      trc(String(i) + ":" + String(ReceivedSignal[i][0])+"/"+String(ReceivedSignal[i][1]));
    }
}

int getMin(){
  unsigned int minimum = ReceivedSignal[0][1];
  int minindex=0;
  for (int i = 0; i < array_size; i++)
  {
    if (ReceivedSignal[i][1] < minimum) {
      minimum = ReceivedSignal[i][1];
      minindex = i;
    }
  }
  return minindex;
}

bool isAduplicate(unsigned long value){
  trc(F("isAdupl?"));
  // check if the value has been already sent during the last time_avoid_duplicate
  for (int i = 0; i < array_size;i++){
  if (ReceivedSignal[i][0] == value){
        unsigned long now = millis();
        if (now - ReceivedSignal[i][1] < time_avoid_duplicate){ // change
        trc(F("no pub. dupl"));
        return true;
      }
    }
  }
  return false;
}

void receivingMQTT(char * topicOri, char * datacallback) {

  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject& jsondata = jsonBuffer.parseObject(datacallback);
  
  if (strstr(topicOri, subjectMultiGTWKey) != NULL) // storing received value so as to avoid publishing this value if it has been already sent by this or another OpenMQTTGateway
  {
    trc(F("Store str"));
    unsigned long data = 0;
    #ifdef jsonPublishing
      if (jsondata.success())  data =  jsondata["value"];
    #endif

    #ifdef simplePublishing
      data = strtoul(datacallback, NULL, 10); // we will not be able to pass values > 4294967295
    #endif
    
    if (data != 0) {
      storeValue(data);
      trc(F("JSON str"));
    }
  }

  if (jsondata.success()) { // json object ok -> json decoding
   #ifdef ZgatewayPilight // ZgatewayPilight is only defined with json publishing
     MQTTtoPilight(topicOri, jsondata);
   #endif
   #ifdef jsonReceiving
    #ifdef ZgatewayLORA
      MQTTtoLORA(topicOri, jsondata);
    #endif
    #ifdef ZgatewayRF
      MQTTtoRF(topicOri, jsondata);
    #endif
    #ifdef ZgatewayRF315
      MQTTtoRF315(topicOri, jsondata);
    #endif
    #ifdef ZgatewayRF2
      MQTTtoRF2(topicOri, jsondata);
    #endif
    #ifdef Zgateway2G
      MQTTto2G(topicOri, jsondata);
    #endif
    #ifdef ZgatewaySRFB
      MQTTtoSRFB(topicOri, jsondata);
    #endif
    #ifdef ZgatewayIR
      SERIALtoIR(topicOri, jsondata);
    #endif
    #ifdef ZgatewayRFM69
      MQTTtoRFM69(topicOri, jsondata);
    #endif
    #ifdef ZgatewayBT
      MQTTtoBT(topicOri, jsondata);
    #endif
   #endif
    #ifdef ZactuatorONOFF // outside the jsonpublishing macro due to the fact that we need to use simplepublishing with HA discovery
      MQTTtoONOFF(topicOri, jsondata);
    #endif
    digitalWrite(led_send, HIGH);

    #ifdef ZactuatorFASTLED
      MQTTtoFASTLEDJSON(topicOri, jsondata);
    #endif


  } else { // not a json object --> simple decoding
   #ifdef simpleReceiving
      #ifdef ZgatewayLORA
        MQTTtoLORA(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRF
        MQTTtoRF(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRF315
        MQTTtoRF315(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRF2
        MQTTtoRF2(topicOri, datacallback);
      #endif
      #ifdef Zgateway2G
        MQTTto2G(topicOri, datacallback);
      #endif
      #ifdef ZgatewaySRFB
        MQTTtoSRFB(topicOri, datacallback);
      #endif
      #ifdef ZgatewayIR
        SERIALtoIR(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRFM69
        MQTTtoRFM69(topicOri, datacallback);
      #endif
  #endif
  #ifdef ZactuatorONOFF
    MQTTtoONOFF(topicOri, datacallback);
  #endif
  digitalWrite(led_send, HIGH);

  #ifdef ZactuatorFASTLED
    MQTTtoFASTLED(topicOri, datacallback);
  #endif

  }
//YELLOW OFF
digitalWrite(led_send, HIGH);
}


//================================ESPNow function starts==========
    


void receiveCallBackFunction(uint8_t *senderMac, uint8_t *incomingData, uint8_t len) {
    memcpy(&myData, incomingData, sizeof(myData));
    
    Serial.print(myData.text);
    Serial.print("  Time : ");
    Serial.print(myData.time);
    Serial.print("  device : ");
    Serial.print (myData.device, DEC);
    Serial.print("  Temperature : ");
    Serial.print (myData.temperature, DEC);
    Serial.print("  Humidity : ");
    Serial.print (myData.humidity, DEC);
    Serial.print("  Pressure : ");
    Serial.print (myData.pressure, DEC);
    Serial.print("  Battery : ");
    Serial.print (myData.battery, DEC);
    Serial.print("  Light : ");
    Serial.println (myData.light, DEC);

    
/*
    Serial.print("  Battery : ");
    for (byte n = 0; n < 1; n++) {
    Serial.print (senderMac[3], DEC);
    }

    Serial.print("  Light : ");
    for (byte n = 0; n < 1; n++) {
    Serial.print (senderMac[4], DEC);

    } 

    Serial.print("  MsgLen ");
    Serial.print(len);
    Serial.print("  Text ");
    Serial.print(myData.text);
    */
   
    Serial.println();
}

//================================ESPNow function ends==========

//================================Probe Request function starts==========
   
void onProbeRequest(const WiFiEventSoftAPModeProbeRequestReceived& dataReceived) {
//  if (dataReceived.mac[5] != CONTROLLER_ID) return;
  
   Serial.print("Probe Request:- ");
   
  //if (dataReceived.mac[5] == 0x01) {
   // if (WiFi.hostname() == WiFi.hostname()) {
    //digitalWrite(SWITCH, HIGH);
    Serial.print(" Device ID:  ");
    Serial.print(dataReceived.mac[5],DEC);
    Serial.print(" Temperature:  ");
    Serial.print(dataReceived.mac[0],DEC);
    Serial.print(" Humidity:  ");
    Serial.print(dataReceived.mac[1],DEC);
    Serial.print(" Pressure:  ");
    Serial.print(dataReceived.mac[2],DEC);
    Serial.print(" Battery:  ");
    Serial.print(dataReceived.mac[3],DEC);
    Serial.print(" Light:  ");
    Serial.println(dataReceived.mac[4],DEC);
 //   } 
  // else {
   // digitalWrite(SWITCH, LOW);
  //  Serial.println("Data Error");
 // }

 
}

//================================Probe Request function ends==========
