#include "Adafruit_BMP3XX.h"
#include "Adafruit_SHT31.h"
#include "RH_RF69.h"
#include "Seeed_SHT35.h"
#include "AS3935I2C.h"
#include "Adafruit_SleepyDog.h"

//load bosch library and set pins
Adafruit_BMP3XX bmp; // I2C

//lightning detector 
#define PIN_IRQ 6

//set frequency, pins and load rf69 library
#define RF69_FREQ 869 // set frequency
#define RFM69_CS      4 //pins
#define RFM69_INT     7 //pins
#define RFM69_RST     5 //pins
#define LED           13 //not used but class needs it
#define TX_POWER      14 //tx power
RH_RF69 rf69(RFM69_CS, RFM69_INT);

#define SDAPIN  11
#define SDLPIN  12
SHT35 sht35sensor(SDLPIN);

//load lightning sensor class
AS3935I2C as3935(AS3935I2C::AS3935I2C_A11, PIN_IRQ);
//this value will be set to true by the AS3935 interrupt service routine.
volatile bool interrupt_ = false;


void setup() { //start setup
  //open serial for debugging
  Serial.begin(115200);
  //while (!Serial) { delay(1); }

    //print header
  Serial.println("BMP388-SHT35D-RFM69-AS3935-Custom Weatherstation");
  int countdownMS = Watchdog.enable(6000); //set watchdog for setup mode
  Serial.print("Watchdog: ");
  Serial.print(countdownMS, DEC);
  Serial.println(" ms!");

  //blinkie blink
  pinMode(LED_BUILTIN, OUTPUT);
  
  //lightning sensor
  pinMode(PIN_IRQ, INPUT);

///--------------------------------------AS3935----------------------------------------------------///
  
  Wire.begin();

  if (!as3935.begin())
  {
    Serial.println("AS3935: begin() failed. Check the I2C address passed to the AS3935I2C constructor. ");
    blinker(10);
    while (1);
  }

  //check I2C connection.
  if (!as3935.checkConnection())
  {
    Serial.println("AS3935: checkConnection() failed. check your I2C connection and I2C Address. ");
    blinker(10);
    while (1);
  }
  else
    Serial.println("AS3935: I2C connection check passed. ");

  int32_t frequency = 0;
  if (!as3935.calibrateResonanceFrequency(frequency))
  {
    Serial.println("AS3935: Resonance Frequency Calibration failed. ");
    blinker(10);
    while (1);
  }
  else
    Serial.println("AS3935: Resonance Frequency Calibration passed. ");
  
  Serial.print("AS3935: Resonance Frequency is "); Serial.print(frequency); Serial.println("Hz");

  //calibrate the RCO.
  if (!as3935.calibrateRCO())
  {
    Serial.println("AS3935: RCO Calibration failed. ");
    blinker(10);
    while (1);
  }
  else
    Serial.println("AS3935: RCP Calibration passed. ");

  //set the analog front end to 'indoors'
  as3935.writeAFE(AS3935MI::AS3935_OUTDOORS);

  //set default value for noise floor threshold
  as3935.writeNoiseFloorThreshold(AS3935MI::AS3935_NFL_2);

  //set the default Watchdog Threshold
  as3935.writeWatchdogThreshold(AS3935MI::AS3935_WDTH_2);

  //set the default Spike Rejection 
  as3935.writeSpikeRejection(AS3935MI::AS3935_SREJ_2);

  //write default value for minimum lightnings (1)
  as3935.writeMinLightnings(AS3935MI::AS3935_MNL_1);

  //do not mask disturbers
  as3935.writeMaskDisturbers(false);

  //the AS3935 will pull the interrupt pin HIGH when an event is registered and will keep it 
  //pulled high until the event register is read.
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), AS3935ISR, RISING);

///---------------------------------------SHT35----------------------------------------------------///

  if(sht35sensor.init())
  {
    Serial.println("SHT35D: Sensor init failed!");
    blinker(9);
    while(1);
  }

///---------------------------------------RADIO---------------------------------------------------///

  // init the rf69 radio
  resetradio();
  if (!rf69.init()) {
    Serial.println("RFM69: radio init failed");
    blinker(8);
    while (1);
  }
  else {
    Serial.println("RFM69: radio init OK!");
  }

  // set rf69 frequency
  if (!rf69.setFrequency(RF69_FREQ)) {
    blinker(8);
    Serial.println("RFM69: SetFrequency failed");
  }

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  //set TX power and print status
  rf69.setTxPower(TX_POWER, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  Serial.print("RFM69: radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz"); Serial.print((int)TX_POWER); Serial.println(" dBi"); Serial.println();

///--------------------------------------BMP------------------------------------------------------///

  // check if bosch sensor is working
  if (!bmp.begin()) {
    Serial.println("Could not find BMP3xx");
    blinker(7);
    while (1);
  }
  else {
    //setup oversampling and filter initialization for bosch sensor
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING); //lowest possible
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X); //highest possible
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3); //1,3,7,15,31,63,127 or off
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  Serial.println("Watchdog: disabled for setup");
  Watchdog.disable(); //disable watchdog since we passed setup
  
} //end setup


int countdownMS = Watchdog.enable(16000); //set watchdog for setup mode

int loopcount = 0;
///-------------------------------------LOOP------------------------------------------------------///
void loop() {

  Serial.print("Watchdog: ");
  Serial.print(countdownMS, DEC);
  Serial.println(" ms!");
 
 //set AS3935 counter defaults
 int strikecount = 0;
 int distance = 0;
 
 //SHT35
 float temp,hum;
 if(NO_ERROR!=sht35sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH,&temp,&hum))
 {
   Serial.println("SHT35D: Read temperature failed!");
 }
 else
 {
  Serial.print("SHT35D: Temperature:"); Serial.println(temp);
  Serial.print("SHT35D: Humidity:"); Serial.println(hum);
 }

 //BMP388
 bmp.performReading();
 
 Serial.print("BMP: Temperature = "); Serial.print(bmp.temperature); Serial.println(" *C");
 Serial.print("BMP: Air Pressure = "); Serial.print(bmp.pressure / 100.0); Serial.println(" hPa");

 //AS3935
 if (interrupt_)
 {
  delay(2);

  //reset the interrupt variable
  interrupt_ = false;

  //query the interrupt source from the AS3935
  uint8_t event = as3935.readInterruptSource();

  //send a report if the noise floor is too high. 
  if (event == AS3935MI::AS3935_INT_NH)
  {
    Serial.println("AS3935: Noise floor too high, attempting to increase noise floor threshold. ");
    //if the noise floor threshold setting is not yet maxed out, increase the setting.
    //note that noise floor threshold events can also be triggered by an incorrect
    //analog front end setting.
    if (as3935.increaseNoiseFloorThreshold())
      Serial.println("AS3935: Increased noise floor threshold");
    else
      Serial.println("AS3935: Noise floor threshold already at maximum");
  }

  //send a report if a disturber was detected. if disturbers are masked with as3935.writeMaskDisturbers(true);
  //this event will never be reported.
  else if (event == AS3935MI::AS3935_INT_D)
  {
    Serial.println("AS3935: Disturber detected, attempting to increase noise floor threshold.");
    //increasing the Watchdog Threshold and / or Spike Rejection setting improves the AS3935s resistance 
    //against disturbers but also decrease the lightning detection efficiency (see AS3935 datasheet)
    uint8_t wdth = as3935.readWatchdogThreshold();
    uint8_t srej = as3935.readSpikeRejection();

    if ((wdth < AS3935MI::AS3935_WDTH_10) || (srej < AS3935MI::AS3935_SREJ_10))
    {
      //alternatively increase spike rejection and watchdog threshold 
      if (srej < wdth)
      {
        if (as3935.increaseSpikeRejection())
          Serial.println("AS3935: Increased spike rejection ratio");
        else
          Serial.println("AS3935: Spike rejection ratio already at maximum");
      }
      else
      {
        if (as3935.increaseWatchdogThreshold())
          Serial.println("AS3935: Increased watchdog threshold");
        else
          Serial.println("AS3935: Watchdog threshold already at maximum");
      }
    }
    else
    {
      Serial.println("AS3935: Error,watchdog Threshold and Spike Rejection settings are already maxed out.");
    }
  }

  else if (event == AS3935MI::AS3935_INT_L)
  {
    strikecount++;
    distance = as3935.readStormDistance();
    Serial.print("AS3935: Lightning detected! Storm Front is ");
    Serial.print(as3935.readStormDistance());
    Serial.println("KM away.");
  }
}

 if (loopcount < 2) {
   Serial.println("Skip first 2 loops!");
   loopcount++;
 } 
 else {

   float tempout = temp;
   float humout = hum;
   float pressure = bmp.pressure / 100.0;
   float tempin = bmp.temperature;
   if ( (tempout <= 70 ) && (tempout >= -35) && (humout > 0)  && (humout < 105) && (pressure <= 1100) && (pressure >= 850)) {
   //send radio message
   String message = "1:" + String(tempout) + ":" + String(humout) + ":" + String(tempin) + ":" + String(pressure) + ":" + String(strikecount) + ":" + String(distance);
   radiosendmessage(message);
   }
 }

  blinker(2);
  Serial.println("Watchdog: resetting counter");
  Watchdog.reset(); //disable watchdog since we passed setup

 delay(10000);
}

//AS3935 interrupt service routine. this function is called each time the AS3935 reports an event by pulling the IRQ pin high.
void AS3935ISR()
{
  interrupt_ = true;
}

// start rf69 radio
void resetradio() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
}

void radiosendmessage(String input) {
  
  // convert to char
  const char* message = input.c_str();
  rf69.send((uint8_t *)message, strlen(message));
  rf69.waitPacketSent();
  Serial.print("Sending radio packet: "); Serial.println(message);
  
}

void blinker(int count)
{
  while(count > 0 ) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(180);
    digitalWrite(LED_BUILTIN, LOW);
    delay(180);
    count = count -1;
  }
}
