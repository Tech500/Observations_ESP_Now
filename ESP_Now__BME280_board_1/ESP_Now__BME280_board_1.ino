/*****************************************************************

  ESP_Now__BME280_board_1.ino  12/18/2023 @ 21.10 EST
  
  William Lucid

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

float temp(NAN), temperature, hum(NAN), pres(NAN), heatIndex, dewPoint, absHum, altitude, seaLevel;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x3C, 0xE9, 0x0E, 0x85, 0x27, 0x40}; 

float currentPressure;
float pastPressure;
float difference;  //change in barometric pressure drop; greater than .020 inches of mercury.
float heatId;      //Conversion of heatIndex to Farenheit
float dewPt;       //Conversion of dewPoint to Farenheit
float altFeet;     //Conversion of altitude to Feet


// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 10
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

//Structure example to send data
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

//Create a struct_message called myData
struct_message myData;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

unsigned int readingId;  

uint8_t data = 0;
// send data
void sendData() {
  data++;
  const uint8_t *peer_addr = slave.peer_addr;
  //Serial.print("Sending: "); Serial.println(myData);
   
  //Set values to send
  myData.id = BOARD_ID;
  myData.temp = temperature;
  myData.heat = heatId;     
  myData.hum = hum;
  myData.dew = dewPt;
  myData.press = currentPressure;
  myData.readingId = readingId++;

  Serial.println("");
  printData();
  Serial.println("");

  //Send message via ESP-NOW
  uint8_t data[sizeof(struct_message)];
  memcpy(data, &myData, sizeof(struct_message));
  esp_err_t result = esp_now_send(peer_addr, data, sizeof(data));


  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

//WiFiClient client;

#define white_LED  25

/*
//Sleep bme280
#define BME280_ADDRESS 0x76
#define BME280_CTRL_MEAS_REG 0xF4
#define BME280_SLEEP_MODE 0x00
//Wake BME280
#define BME280_CTRL_MEAS_REG 0xF4
#define BME280_NORMAL_MODE 0x03
*/

char strftime_buf[64];

String dtStamp(strftime_buf);

void setup() {

  startMicros = micros();

  //wakeBME280();

  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  while(!Serial);

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

    /*
    First we configure the wake up source
    We set our ESP32 to wake up every 5 seconds
  */

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  /*
    Next we decide what all peripherals to shut down/keep on
    By default, ESP32 will automatically power down the peripherals
    not needed by the wakeup source, but if you want to be a poweruser
    this is for you. Read in detail at the API docs
    http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
    Left the line commented as an example of how to configure peripherals.
    The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
    Now that we have setup a wake cause and if needed setup the
    peripherals state in deep sleep, we can now start going to
    deep sleep.
    In the case that no wake up sources were provided but deep
    sleep was started, it will sleep forever unless hardware
    reset occurs.
  */   
   
}

void loop() {

  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
      sendData();
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }

  // wait for 3seconds to run the logic again
  delay(3000);
   
   //BME280_Sleep();

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

  //Set values to send
  myData.id = BOARD_ID;
  myData.temp = temperature;
  myData.heat = heatId;     
  myData.hum = hum;
  myData.dew = dewPt;
  myData.press = currentPressure;
  myData.readingId = readingId++;

}


/*
void BME280_Sleep() 
{

  // Write sleep mode to control register
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(BME280_CTRL_MEAS_REG);
  Wire.write(BME280_SLEEP_MODE);
  Wire.endTransmission();

  Serial.println("BME280 is now in sleep mode!");


}
*/

void goToDeepSleep()
{  
  Serial.println("Going to sleep now");
  
  elapsedMicros = micros() - startMicros;

  digitalWrite(white_LED,LOW);

  Serial.flush();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  //adc_power_off();
  esp_wifi_stop();
  esp_bt_controller_disable();

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
  
}

void printData()
{
  
  Serial.println(myData.id);
  Serial.println(myData.temp);
  Serial.println(myData.heat);     
  Serial.println(myData.hum);
  Serial.println(myData.dew);
  Serial.println(myData.press);

  delay(2000);
}

/*
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
*/
