#include <SPI.h>
#include <RH_RF69.h>
#include <WiFiNINA.h>

#define SECRET_SSID "xxxxx"
#define SECRET_PASS "xxxxx"

// wifi setup
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;

//send data
char server[] = "xxx.xxx.xxx.xxx"; //server IP
WiFiClient client;

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 868.0
#define RFM69_INT     7
#define RFM69_CS      6
#define RFM69_RST     5
#define LED           13
#define TX_POWER      20

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() 
{
  Serial.begin(9600);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  // rf69 manual reset
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  //check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();
}

void loop() {

 char postdata[100];
 int siteid = 1;
 
 if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      
      if (!len) return;
      buf[len] = 0;

      Serial.print("Message: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);

      char* data;
      data = (char*)buf;
      char separator[] = ":";
      char *array[4];
      int i=0;
      array[i] = strtok(data,separator);
      if( array[i][0] == '1' && array[i][1] == 0 ) {
        while( i < sizeof(array) / sizeof(*array) && array[i] != NULL)  { 
          array[i] = strtok(NULL,separator);
          Serial.println(array[i]);
          i++ ;
        }
      }
    
      char* t = array[0];
      char* h = array[1];
      char* p = array[2];
      Serial.print("Temperature =  "); Serial.println(t);
      Serial.print("Rel. Humidity =  "); Serial.println(h);
      Serial.print("Pressure =  "); Serial.println(p);
      sprintf(postdata, "{\"siteid\":%i,\"TempOut\":%s,\"HumOut\":%s,\"Pressure\":%s}", siteid,t,h,p);
    
    }

    Serial.println("connecting to server");
    if (client.connect(server, 4444)) {
      // Make a HTTP request:
      Serial.print("sending json data: ");
      Serial.println(postdata);
      client.println("POST index.php/upload/weather/");
      client.println("Content-Type: application/json");
      client.println("Accept: application/json");
      client.println(postdata);
      //client.print("content-length: ");
      //client.println(strlen(postdata));
      //client.print("Host:");
      //client.println(server);
      client.println("Connection: close");
    }
    else {
       Serial.println("unable to connect to server");
    }
     
  }
}


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
