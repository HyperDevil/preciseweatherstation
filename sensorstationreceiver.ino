#include <RH_RF69.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"
#include <Adafruit_Sensor.h>
#include "Seeed_SHT35.h"
#include <utility/wifi_drv.h>

/************ Secrets setup for Wifi***************/
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

/************ Initialize classes ***************/
int status = WL_IDLE_STATUS;
WiFiSSLClient client; //SSL Support
MqttClient mqttClient(client); //pass ssl support to mqtt

/************ Radio setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 868.0
#define RFM69_INT     7
#define RFM69_CS      6
#define RFM69_RST     5
#define LED           13
#define TX_POWER      18
RH_RF69 rf69(RFM69_CS, RFM69_INT);

#define SDAPIN  A4
#define SCLPIN  A5
#define RSTPIN  2
SHT35 sensor(SCLPIN);

const char MQTT_SERVER[] = SECRET_BROKER;
#define MQTT_PORT 8883
const char MQTT_PASSWORD[] = SECRET_PASSWORD;
const char MQTT_USERNAME[] = SECRET_MQTT_USER;


//start setup
void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  delay(5000); //so we can catch the first messages on usb-to-serial

  Serial.println("Booting Arduino MKR 1010 Wifi....");
  
  delay(100);
  Serial.println("Resetting RFM69 module....");

  // wifi leds
  WiFiDrv::pinMode(25, OUTPUT); //GREEN
  WiFiDrv::pinMode(26, OUTPUT); //RED
  WiFiDrv::pinMode(27, OUTPUT); //BLUE

  // rf69 manual reset
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  //sht35 sensor test
  if(sensor.init())
  {
   Serial.println("SHT-35 sensor error!");
   while(1);
  }

  //check if the radio is available
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  else {
    Serial.println("RFM69 radio init OK!");
  }

  //try to set the radio frequency
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("RFM69 set frequency failed");
  }
  else {
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

} //end setup


// start permanent loop
void loop() {

   if (WiFi.status() != WL_CONNECTED) {
     connectWiFi();
   }

   if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
   }

   //avoid being disconnected
   mqttClient.poll();

   //sht35 values
   u16 value=0;
   u8 data[6]={0};
   float sht35temp,sht35hum;

   if(NO_ERROR!=sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH,&sht35temp,&sht35hum))
   {
      Serial.println("SHT35 reading temperature failed!");
   }
   else
   {
      if (sht35temp > -30 && sht35temp < 70 && sht35hum > 0 && sht35hum < 110) //we need to check for sanity here
      { 
         publishMessage(sht35temp,(char *)"weather/**/inside_temperature");
         publishMessage(sht35hum,(char *)"weather/**/inside_humidity");
         
         //blink LED
         WiFiDrv::digitalWrite(26, HIGH);
         delay(150);
         WiFiDrv::digitalWrite(26, LOW);
         delay(150);
      }
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

      //blink LED
      WiFiDrv::digitalWrite(27, HIGH);
      delay(150);
      WiFiDrv::digitalWrite(27, LOW);
      
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
      float t2 = atof(array[1]);
      float h = atof(array[2]);
      float p = atof(array[3]);

      //check if we have all the data
      if (t1 != '\0' && t2 != '\0' && h != '\0' && p != '\0') {
         //send ack back to radio that all is OK
         delay(150);
         char radiopacket[5] = "ACK";
         rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
         
         //blink LED
         WiFiDrv::digitalWrite(25, HIGH);
         delay(150);
         WiFiDrv::digitalWrite(25, LOW);
         
         if (t1 > -50 && t1 < 80) { publishMessage(t1,(char *)"weather/**/outside_temperature"); } //check for sanity
         if (t2 > -10 && t2 < 80) { publishMessage(t2,(char *)"weather/**/outside_enclosure_temperature"); } //check for sani
         if (h > 0 && h < 110) { publishMessage(h,(char *)"weather/**/outside_relative_humidity"); } //check for sanity
         if (p > 900 && p < 1100) { publishMessage(p,(char *)"weather/**/outside_pressure"); } // check for sanity
         publishMessage(rssi,(char *)"weather/**/rssi_received"); //we dont care about this really its already set
      }

    }
  } 

  delay(25000);
} //end void loop


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

void connectMQTT() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(MQTT_SERVER);

  mqttClient.setUsernamePassword(MQTT_USERNAME,MQTT_PASSWORD);
  while (!mqttClient.connect(MQTT_SERVER,MQTT_PORT)) {

   //also check wifi
   if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
   }
    
    // failed, retry
    Serial.print(".");
    delay(2000);
  }

  Serial.println();
  Serial.println("Connected to MQTT broker");
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
  mqttClient.beginMessage(topic);
  mqttClient.print(value);
  mqttClient.print(millis());
  mqttClient.endMessage();
  Serial.print("Published message: "); Serial.print(topic); Serial.print(" "); Serial.println(value);
}
