#include "Adafruit_BMP3XX.h"
#include "Adafruit_SHT31.h"
#include "RH_RF69.h"

//load sensirion library
#define SHT31_ADDR 0x44 // Set to 0x45 for alternate i2c addr
Adafruit_SHT31 sht31 = Adafruit_SHT31();

//load bosch library and set pins
Adafruit_BMP3XX bmp; // I2C

//set frequency, pins and load rf69 library
#define RF69_FREQ 868 // set frequency
#define RFM69_CS      4 //pins
#define RFM69_INT     3 //pins
#define RFM69_RST     2 //pins
#define LED           13 //not used but class needs it
#define TX_POWER      14 //tx power
RH_RF69 rf69(RFM69_CS, RFM69_INT);
#define STATUS_LED    9 

// start rf69 radio
void resetradio() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
}

void setup() {
  //open serial for debugging
  Serial.begin(9600);
  //while (!Serial) { delay(1); }

  //print header
  Serial.println("BMP388-SHT31D-RFM69 Custom Weatherstation");

  // initialize pin 9 as status LED pin
  pinMode(STATUS_LED, OUTPUT);
  //digitalWrite(STATUS_LED, LOW);

  // init the rf69 radio
  resetradio();
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
    digitalWrite(STATUS_LED, HIGH);
    delay(1000);
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
    digitalWrite(STATUS_LED, HIGH);
    delay(1000);
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
    digitalWrite(STATUS_LED, HIGH);
    while (1);
  }
  else {
    Serial.println("RFM69 radio init OK!");
  }

  // set rf69 frequency
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // The encryption key has to be the same as the one in the server, this is default
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  //set TX power and print status
  rf69.setTxPower(TX_POWER, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz"); Serial.print((int)TX_POWER); Serial.println(" dBi"); Serial.println();

  // check if bosch sensor is working
  if (!bmp.begin()) {
    Serial.println("Could not find BMP3xx");
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
    digitalWrite(STATUS_LED, HIGH);
    delay(1000);
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
    digitalWrite(STATUS_LED, HIGH);
    while (1);
  }
  else {
    //setup oversampling and filter initialization for bosch sensor
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING); //lowest possible
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X); //highest possible
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3); //1,3,7,15,31,63,127 or off
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  //check if sensirion sensor is working
  if (! sht31.begin(SHT31_ADDR)) {
    Serial.println("Could not find SHT31");
        digitalWrite(STATUS_LED, LOW);
    delay(1000);
    digitalWrite(STATUS_LED, HIGH);
    delay(1000);
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
    digitalWrite(STATUS_LED, HIGH);
    while (1);
  }
  else {
    //disable the internal heater on the sensirion sensor
    sht31.heater(false);
  }

}

//loop counter
int count = 0;

void loop() {

  bmp.performReading();
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  float p = bmp.pressure / 100.0;
 
  Serial.print("BMP Temperature = "); Serial.print(bmp.temperature); Serial.println(" *C");
  Serial.print("BMP Air Pressure = "); Serial.print(p); Serial.println(" hPa");
  Serial.print("SHT31 Temperature *C = "); Serial.println(t);
  Serial.print("SHT31 Rel. Humidity % = "); Serial.println(h);

  // skip first 1 datasets due to sensor warm-up
  if (count < 1) {
    count++;
    Serial.println("Skipping first dataset...");
  }
  else {
    //check data sanity
    if ( (t <= 70 ) && (t >= -35) && (h > 0)  && (h < 105) && (p <= 1100) && (p >= 850)) {
      String sendstring = "1:" + String(t) + ":" + String(h) + ":" + String(p);
      char radiopacket[22];
      sendstring.toCharArray(radiopacket,22);
      Serial.print("Sending "); Serial.println(radiopacket);
      rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf69.waitPacketSent();
      digitalWrite(STATUS_LED, HIGH);
      delay(150);
      digitalWrite(STATUS_LED, LOW);
    }
    else {
      Serial.println("Data is not sane!");
    }
  }

  Serial.println();
  delay(30000);
}
