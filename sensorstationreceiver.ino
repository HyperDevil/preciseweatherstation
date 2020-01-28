#include <RH_RF69.h>
#include <WiFiNINA.h>
#include <ArduinoBearSSL.h>
#include <ArduinoMqttClient.h>
#include <ArduinoECCX08.h>
#include "arduino_secrets.h"

/************ Secrets setup ***************/
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
const char broker[] = SECRET_BROKER;
const char* certificate = SECRET_CERTIFICATE;

/************ Initialize classes ***************/
int status = WL_IDLE_STATUS;
WiFiClient client;
BearSSLClient sslClient(client);
MqttClient mqttClient(sslClient);

/************ Radio setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 868.0
#define RFM69_INT     7
#define RFM69_CS      6
#define RFM69_RST     5
#define LED           13
#define TX_POWER      18
RH_RF69 rf69(RFM69_CS, RFM69_INT);

unsigned long lastMillis = 0;

void setup() 
{
  Serial.begin(9600);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("Booting Arduino MKR 1010 Wifi....");

  Serial.println("Resetting RFM69 module....");
  // rf69 manual reset
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!ECCX08.begin()) {
    Serial.println("No ECCX08 present!");
    while (1);
  }

  //get time from wifi chip
  ArduinoBearSSL.onGetTime(getTime);
  //get certificate from slot 0
  sslClient.setEccSlot(0, certificate);
  //mqttClient.onMessage(onMessageReceived);
  
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("RFM69 set frequency failed");
  }

  Serial.print("RFM69 TX Power: "); 
  Serial.print(TX_POWER);
  Serial.println(" dBm");
  rf69.setTxPower(TX_POWER, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.print("RFM69 radio @");  
  Serial.print((int)RF69_FREQ);  
  Serial.println(" MHz");
}

// start permanent loop
void loop() {

   if (WiFi.status() != WL_CONNECTED) {
     connectWiFi();
   }

   if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
   }

   if (rf69.available()) {
    
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {

      if (!len) return;
      
      buf[len] = 0;

      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);

      char *array[4];
      char separator[] = ":";
      char *segmentPointer = strtok((char*)buf, separator);
      int i = 0;
      if ((segmentPointer[0] == '1') && (segmentPointer[1] == '\0')) {
        segmentPointer = strtok(NULL, separator);
        while (segmentPointer != NULL) {
          array[i] = segmentPointer;
          i++;
          segmentPointer = strtok(NULL, separator);
        }
      }

      float rssi = (float)rf69.lastRssi();
      float t1 = atof(array[0]);
      //char* t1 = array[0];
      float t2 = atof(array[1]);
      float h = atof(array[2]);
      float p = atof(array[3]);
      Serial.print("Temperature Outside= "); Serial.print(t1);
      Serial.print(" Temperature Enclosure= "); Serial.print(array[1]);
      Serial.print(" Rel. Humidity= "); Serial.print(array[2]);
      Serial.print(" Pressure= "); Serial.println(array[3]);
      
      if (t1 != '\0' && t2 != '\0' && h != '\0' && p != '\0') {
         publishMessage(t1,(char *)"yourtopic/outside_temperature");
         publishMessage(t2,(char *)"yourtopic/outside_enclosure_temperature");
         publishMessage(h,(char *)"yourtopic/outside_relative_humidity");
         publishMessage(p,(char *)"yourtopic/outside_pressure");
         publishMessage(rssi,(char *)"yourtopic/rssi_received");
      }
    
     }
   } 
}//end void loop


void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

unsigned long getTime() {
  // get the current time from the WiFi module  
  return WiFi.getTime();
}

void connectMQTT() {
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883)) {
    // failed, retry
    Serial.print(".");
    delay(3000);
  }
  Serial.println();

  Serial.println("Connected to AWS IoT");
}

void connectWiFi() {
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(ssid);
  Serial.print(" ");

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(3000);
  }
  Serial.println();
  printCurrentNet();
  printWifiData();
}

void publishMessage(float value, char* topic) {
// send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(topic);
  mqttClient.print(value);
  mqttClient.print(millis());
  mqttClient.endMessage();
  Serial.print("Published message: "); Serial.print(topic); Serial.print(" "); Serial.println(value);
}
