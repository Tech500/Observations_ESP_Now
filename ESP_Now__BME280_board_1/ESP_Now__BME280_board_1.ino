/*****************************************************************

  ESP_Now__BME280_board_1.ino  12/18/2023 @ 21.10 EST
  
  William Lucid and Google's Bard

  Board:  ESP32 Pico Kit v4.1
  BME280 Breakout board
  
  
*/////////////////////////////////////////////////////////////////

#include <esp_now.h>
#include <WiFi.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>
#include <BME280I2C.h>                //Use the Arduino Library Manager, get BME280 by Tyler Glenn
#include <EnvironmentCalculations.h>  //Use the Arduino Library Manager, get BME280 by Tyler Glenn
#include <Wire.h>    //Part of version 1.0.4 ESP32 Board Manager install  -----> Used for I2C protocol



//BME280

// Assumed environmental values:
float referencePressure = 1021.1;   // hPa local QFF (official meteor-station reading) ->  KEYE, Indianapolis, IND
float outdoorTemp = 35.6;           // °F  measured local outdoor temp.
float barometerAltitude = 250.698;  // meters ... map readings + barometer position  -> 824 Feet  Garmin, GPS measured Altitude.

BME280I2C::Settings settings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_16,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x76);

BME280I2C bme(settings);

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x3C, 0xE9, 0x0E, 0x85, 0x27, 0x40}; //board_2 mac address

// Define variables to store incoming readings
float temp(NAN), temperature, hum(NAN), pres(NAN), heatIndex, dewPoint, absHum, altitude, seaLevel;

float currentPressure;
float pastPressure;
float difference;  //change in barometric pressure drop; greater than .020 inches of mercury.
float heatId;      //Conversion of heatIndex to Farenheit
float dewPt;       //Conversion of dewPoint to Farenheit
float altFeet;     //Conversion of altitude to Feet


// Global copy of slave
//esp_now_peer_info_t slave;
#define CHANNEL 10
//#define PRINTSCANRESULTS 0
//#define DELETEBEFOREPAIR 0

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

// Variable to store if sending data was successful
String success;

//Board_1 to Board 2
//Must match the receiver structure
typedef struct struct_message {
  int id;
  float temp;
  float heat;
  float hum;
  float dew;
  float press;     
  unsigned int readingId;
} struct_message;

// Create a struct_message called relayReadings to hold relay readings
struct_message bme280Readings;  //BME280 readings from sensor

// Create a struct_message called bmeReadings to send BME280 readings
struct_message incomingReadings1;  //from board_1 to board_2

esp_now_peer_info_t peerInfo;

unsigned int readingId; 

// callback when data is sent from board_1 to board_2
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings1, incomingData, sizeof(incomingReadings1));
  Serial.print("Bytes received: ");

  getWeatherData();

  Serial.println(len);
  //BOARD_ID = incomingReadings1.id;
  temperature = incomingReadings1.temp;
  heatId = incomingReadings1.heat;     
  hum = incomingReadings1.hum;
  dewPt = incomingReadings1.dew;
  currentPressure = incomingReadings1.press;
  readingId = incomingReadings1.readingId++;

  printData();
}

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 120      /* Time ESP32 will go to sleep (in seconds) */

unsigned long delayTime;

RTC_DATA_ATTR int bootCount = 0;

RTC_DATA_ATTR float elapsedMicros = 0;

RTC_DATA_ATTR float seconds = 0;

float startMicros;

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n\n", wakeup_reason); break;
  }

}

WiFiClient client;

#define white_LED  25


//Sleep bme280
#define BME280_ADDRESS 0x76
#define BME280_CTRL_MEAS_REG 0xF4
#define BME280_SLEEP_MODE 0x00
//Wake BME280
#define BME280_CTRL_MEAS_REG 0xF4
#define BME280_NORMAL_MODE 0x03

char strftime_buf[64];

String dtStamp(strftime_buf);

void setup() {

  startMicros = micros();

  //wakeBME280();

  Serial.begin(115200);

  while(!Serial);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  //esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  
    // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  //peerInfo.channel = 10;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Wire.begin(21, 22);

  while (!bme.begin())
  {
    Serial.println("");
    Serial.println("Could not find BME280 sensor!");
    delayTime = 1000;
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch (bme.chipModel())  {
    case BME280::ChipModel_BME280:
      Serial.println("");
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("");
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("");
      Serial.println("Found UNKNOWN sensor! Error!");
  }

  getWeatherData();

  pinMode(white_LED,OUTPUT);
  
  digitalWrite(white_LED,HIGH);
 
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  float seconds = elapsedMicros / 1000000;  //Convert to seconds
  Serial.print(" Seconds:  ");
  Serial.print(seconds, 1);
  Serial.println("  Awake time");
  Serial.println("");  

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  delay(5 * 1000);
}

void loop() {

  // Send data before going into deep sleep
  Serial.println("Sending data and going to sleep...");
  // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &bme280Readings, sizeof(bme280Readings));

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
 
  goToDeepSleep();

  Serial.println("This will never be printed");

}

void getWeatherData() 
{
  
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  EnvironmentCalculations::AltitudeUnit envAltUnit = EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit envTempUnit = EnvironmentCalculations::TempUnit_Celsius;

  delay(300);

  /// To get correct local altitude/height (QNE) the reference Pressure
  ///    should be taken from meteorologic messages (QNH or QFF)
  /// To get correct seaLevel pressure (QNH, QFF)
  ///    the altitude value should be independent on measured pressure.
  /// It is necessary to use fixed altitude point e.g. the altitude of barometer read in a map
  absHum = EnvironmentCalculations::AbsoluteHumidity(temp, hum, envTempUnit);
  altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);
  dewPoint = EnvironmentCalculations::DewPoint(temp, hum, envTempUnit);
  heatIndex = EnvironmentCalculations::HeatIndex(temp, hum, envTempUnit);
  seaLevel = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, temp, pres, envAltUnit, envTempUnit);
  heatId = (heatIndex * 1.8) + 32;
  dewPt = (dewPoint * 1.8) + 32;
  altFeet = 843;

  temperature = (temp * 1.8) + 32;  //Convert to Fahrenheit

  currentPressure = seaLevel * 0.02953;  //Convert from hPa to in Hg.
}

void BME280_Sleep() 
{

  // Write sleep mode to control register
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(BME280_CTRL_MEAS_REG);
  Wire.write(BME280_SLEEP_MODE);
  Wire.endTransmission();

  Serial.println("BME280 is now in sleep mode!");
}

void goToDeepSleep()
{  
  Serial.println("Going to sleep now");
  
  elapsedMicros = micros() - startMicros;

  digitalWrite(white_LED,LOW);

  Serial.flush();

  //BME280_Sleep();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  //adc_power_off();
  esp_wifi_stop();
  esp_bt_controller_disable();

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
  
}

void printData() //Sent BME280 Data
{
  
  Serial.println(bme280Readings.id);
  Serial.println(bme280Readings.temp);
  Serial.println(bme280Readings.heat);     
  Serial.println(bme280Readings.hum);
  Serial.println(bme280Readings.dew);
  Serial.println(bme280Readings.press);

  delay(2000);
}

void wakeBME280()
{
  // Wake up the BME280 sensor
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(BME280_CTRL_MEAS_REG);
  Wire.write(BME280_NORMAL_MODE);
  Wire.endTransmission();

  // Delay for sensor to stabilize
  delay(100);

  // Read sensor data here...

  // Delay between readings
  delay(1000);
}