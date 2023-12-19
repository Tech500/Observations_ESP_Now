/*

  ESP_Now__Webserver_Board_2.ino 12/18/2023 @ 20:00 EST
  William Lucid and Google's Bard

*/

// 
// 
//
//   See library downloads for each library license.
//
// 
// 


#include <esp_now.h>
#include "EEPROM.h"  //Part of version 1.0.4 ESP32 Board Manager install
#include <WiFi.h>   //Part of version 1.0.4 ESP32 Board Manager install
#include <HTTPClient.h>  //Part of version 1.0.4 ESP32 Board Manager install
#include <AsyncTCP.h>  //https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>  //https://github.com/me-no-dev/ESPAsyncWebServer
#include <Arduino_JSON.h>
#include <ESPmDNS.h> //Part of version 1.0.4 ESP32 Board Manager install
#include <FTPServer.h>  //https://github.com/nailbuster/esp8266FTPServer  -->Needed for ftp transfers
#include <HTTPClient.h>   //Part of version 1.0.4 ESP32 Board Manager install  ----> Used for Domain Web Interace
#include <WiFiUdp.h>  //1.0.4 ESP32 Board Manager install
#include <sys/time.h>  // struct timeval --> Needed to sync time
#include <time.h>   // time() ctime() --> Needed to sync time
#include "FS.h"
#include "SPIFFS.h"
#include <FTPServer.h>
#include "Update.h"  //1.0.4 ESP32 Board Manager install
#include <ThingSpeak.h>   //https://github.com/mathworks/thingspeak-arduino . Get it using the Library Manager
//#include <TinyGPSPlus.h> //http://arduiniana.org/libraries/tinygpsplus/ Used for GPS parsing
#include <Wire.h>    //Part of version 1.0.4 ESP32 Board Manager install  -----> Used for I2C protocol
#include <Ticker.h>  //Part of version 1.0.4 ESP32 Board Manager install  -----> Used for watchdog ISR
//#include <LiquidCrystal_I2C.h>   //https://github.com/esp8266/Basic/tree/master/libraries/LiquidCrystal optional
#include <rom/rtc.h>
#include "variableInput.h"  //Packaged with project download.  Provides editing options; without having to search 2000+ lines of code.


#import "index1.h"  //Weather HTML; do not remove

#import "index2.h"  //SdBrowse HTML; do not remove

#import "index3.h"  //Graphs HTML; do not remove

#import "index4.h"  //Restarts server; do not remove

#import "index5.h"  //Contactus.HTML; do not remove

#import "index6.h"  //Read File HTML; do not remove

#import "index7.h"  //Video feed HTML; do not remove

void print_reset_reason(int reason)
{
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1,  Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3,  Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4,  Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5,  Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6,  Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9,  RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

void verbose_print_reset_reason(int reason)
{
  switch ( reason)
  {
    case 1  : Serial.println ("Vbat power on reset");break;
    case 3  : Serial.println ("Software reset digital core");break;
    case 4  : Serial.println ("Legacy watch dog reset digital core");break;
    case 5  : Serial.println ("Deep Sleep reset digital core");break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core");break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core");break;
    case 10 : Serial.println ("Instrusion tested to reset CPU");break;
    case 11 : Serial.println ("Time Group reset CPU");break;
    case 12 : Serial.println ("Software reset CPU");break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU");break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println ("NO_MEAN");
  }
}

#define CHANNEL 10

uint8_t broadcast1[] = {0xAC,0x0B,0xFB,0x5C,0xCB,0x80};
//board_1 mac address:  AC:0B:FB:5C:CB:80  ESP32 Pico D4  BME280

uint8_t broadcast2[] = {0x30,0x83,0x98,0xD9,0x1D,0xEC};
//board_3 mac address:  30:83:98:D9:1D:EC    ESP Pico D4  Relay

unsigned int readingId; 

esp_now_peer_info_t peerInfo1;
esp_now_peer_info_t peerInfo2;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  float temp;
  float heat;
  float hum;
  float dew;
  float press;
  int readingId;
} struct_message;

struct_message myData;

float temperature;
float heatId;
float humd;
float dewPt;
float currentPressure;
float pastPressure;  
float difference;

// Relay structure
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message2 {
    int id;
    int ctrl;
    int stat;
    unsigned int readingId;
} struct_message2;

// Create a struct_message called relayReadings to hold relay readings
struct_message2 relayReadings;

//Define variables to hold incoming readings
int controlRelay = 0;  //Open relay contact
int relayStatus = 0;  //Status , relay contact open

// callback when data is sent from Master to Slave   from board_2 to BNE280, board_1
void OnDataSent1(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback when data is sent from Master to Slave from board_2 to relay, board_3
void OnDataSent2(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback when data is recv from BME280, board_1 to board_2
void OnDataRecv1(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  memcpy(&myData, incomingData, sizeof(myData));
  
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  Serial.printf("t value: %4.2f \n", myData.temp);
  Serial.printf("i value: %4.2f \n", myData.heat);
  Serial.printf("h value: %4.2f \n", myData.hum);
  Serial.printf("d value: %4.2f \n", myData.dew);
  Serial.printf("p value: %4.3f \n", myData.press);
  Serial.printf("readingId value: %d \n", myData.readingId++);
  Serial.println();

  readings1();

  printincomingReadings();

  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(myData.readingId++);
  Serial.println("");
}  

// callback when data is recv from relay, board_3 to board_2
void OnDataRecv2(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  memcpy(&relayReadings, incomingData, sizeof(relayReadings));
  
  Serial.printf("Board ID %u: %u bytes\n", relayReadings.id, len);
  Serial.printf("Control: %4f \n", relayReadings.ctrl);
  Serial.printf("Contact: %4f \n", relayReadings.stat);
  Serial.printf("readingId: %d \n", relayReadings.readingId++);
  Serial.println();

  readings2();

  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(relayReadings.readingId++);
  Serial.println("");
}

// Replace with your network credentials (STATION)
const char* ssidStation = "R2D2";
const char* passwordStation = "sissy4357";

WiFiClient client;

IPAddress ipREMOTE;

int connect = 0;
int disconnect = 0;
int count = 1;
int counter = 0;
int brownout = 0;
int softReset = 0;
int powerOn = 0;

void WiFiEvent(WiFiEvent_t event) {
  //Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      //Serial.println("Connected to access point");
      connect = 1;
      disconnect = 0;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      //Serial.println("Disconnected from WiFi access point");
      disconnect = 1;
      if (event == 7) {
        disconnect = 0;
      }
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("\nObtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    default: break;
  }
}

///Are we currently connected?
boolean connected = false;

WiFiUDP udp;
// local port to listen for UDP packets
//const int udpPort = 1337;
char incomingReadingsPacket[255];
char replyPacket[] = "Hi there! Got the message :-)";
//const char * udpAddress1;
//const char * udpAddress2;

FTPServer ftpSrv(SPIFFS);

AsyncWebServer serverAsync(PORT);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)

//OTA Support
//const char* http_username = """"
//const char* http_password = "";
const char* WiFi_hostname = "esp32";

//flag to use from web update to reboot the ESP
bool shouldReboot = false;
int logon;

void onRequest(AsyncWebServerRequest *request)
{
  //Handle Unknown Request
  request->send(404);
}

void onBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  //Handle body
}

void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  //Handle upload
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{
  //Handle WebSocket event
}

//End OEA Support 

Ticker secondTick;
Ticker secondTick2;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile int watchdogCounter;
int totalwatchdogCounter;
int watchDog;

void IRAM_ATTR ISRwatchdog()
{

  portENTER_CRITICAL_ISR(&mux);
  watchdogCounter++;
  delay(100);
  portEXIT_CRITICAL_ISR(&mux);

}

int DOW, MONTH, DATE, YEAR, HOUR, MINUTE, SECOND;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

char strftime_buf[64];

String dtStamp(strftime_buf);

String lastUpdate;

unsigned long delayTime;

#define TZ "EST+5EDT,M3.2.0/2,M11.1.0/2"

/* Two "independant" timed events */
const long eventTime_1 = 5000; //gps
const long eventTime_2 = 1000;   //boot
const long eventTime_3 = 45 * 1000;
const long eventTime_4 = 5 * 60 * 1000;
const long eventTime_5 = 60 * 1000; //in ms

/* When did they start the race? */
unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;
unsigned long previousTime_4 = 0;
unsigned long previousTime_5 = 0;

int lc = 0;
time_t tnow = 0;

int i;

int error = 0;
int flag = 0;
int wait = 0;

int started;   //Used to tell if Server has started

//#define sonalert 9  // pin for Piezo buzzer

#define online 19  //pin for online LED indicator

char* filelist[12];

int counted;

String logging;

char *filename;
char str[] = {0};

String fileRead;
String fn;
String uncfn;
String urlPath; 

char MyBuffer[17];

String PATH;

//String publicIP;   //in-place of xxx.xxx.xxx.xxx put your Public IP address inside quotes

//define LISTEN_PORT;  // in-place of yyyy put your listening port number
// The HTTP protocol uses port 80 by default.

/*
  This is the ThingSpeak channel number for the MathwWorks weather station
  https://thingspeak.com/channels/YourChannelNumber.  It senses a number of things and puts them in the eight
  field of the channel:

  Field 1 - Temperature (Degrees C )
  Field 2 - Humidity (%RH)
  Field 3 - Barometric Pressure (hpa)
  Field 4 - Dewpoint
*/


/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */


//Calibrate rain bucket here
//Rectangle raingauge from Sparkfun.com weather sensors
//float rain_bucket_mm = 0.011*25.4;//Each dump is 0.011" of water
//DAVISNET Rain Collector 2
//float rain_bucket_mm = 0.01*25.4;//Each dump is 0.01" of water  //Convert inch to millmeter (0.01 * 25.4)

// volatiles are subject to modification by IRQs
//volatile unsigned long raintime, rainlast, raininterval, rain, Rainindtime, Rainindlast;  // For Rain
//int addr=0;

#define eeprom_size 512

String eepromstring = "0.00";

unsigned long lastSecond, last5Minutes;
float lastPulseCount;
int currentPulseCount;
float rain5min;
float rainFall;
float rainHour;
float rainDay;
float daysRain;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define FIVEMINUTES (300*1000L)
#define REEDPIN 5   //was 32 Touch pin
#define REEDINTERRUPT 0

volatile int pulseCount_ISR = 0;



void IRAM_ATTR reedSwitch_ISR()
{
  static unsigned long lastReedSwitchTime;
  // debounce for a quarter second = max. 4 counts per second
  if (labs(millis() - lastReedSwitchTime) > 250)
  {
    portENTER_CRITICAL_ISR(&mux);
    pulseCount_ISR++;

    lastReedSwitchTime = millis();
    portEXIT_CRITICAL_ISR(&mux);
  }

}

volatile int cameraCounter;
volatile int cameraPoweroff = 0;

void IRAM_ATTR ISRCamera()
{

  portENTER_CRITICAL_ISR(&mux);

  cameraCounter++;

  if (cameraCounter >= 120)
  {

    cameraPoweroff = 1;

    //switchRelay = 0;  //turn off camera

    secondTick2.detach();

  }
}

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

/*
// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}
*/

/*
// callback when data is recv from Master
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  
  Serial.printf("Board ID %u: %u bytes\n", incomingReadings.id, len);
  Serial.printf("t value: %4.2f \n", incomingReadings.temp);
  Serial.printf("i value: %4.2f \n", incomingReadings.heat);
  Serial.printf("h value: %4.2f \n", incomingReadings.hum);
  Serial.printf("d value: %4.2f \n", incomingReadings.dew);
  Serial.printf("p value: %4.3f \n", incomingReadings.press);
  Serial.printf("readingId value: %d \n", incomingReadings.readingId);
  Serial.println();

  readings();

  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(incomingReadings.readingId);
  Serial.println("");

}
*/

void setup() {

  started = 0;   //Server started

  Serial.begin(115200);

  while(!Serial);

  if (rtc_get_reset_reason(0) == 1)  //VBAT_RESET --brownout restart
  {

    brownout = 1;

    Serial.println("Brownout reset previous boot");

    //powerOn = 1;

  }

  if (rtc_get_reset_reason(0) == 12)  //SOFTWARE_RESET --watchdog restart
  {

    softReset = 1;

    Serial.println("Software reset previous boot");

  }

  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
 // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  
  // Set device as a Wi-Fi Station
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  //esp_now_register_recv_cb(OnDataRecv);

  esp_now_register_send_cb(OnDataSent1);
  esp_now_register_send_cb(OnDataSent2);

  esp_now_register_recv_cb(OnDataRecv1);
  esp_now_register_recv_cb(OnDataRecv2);

  // Register the remote nodes
 
  memcpy(peerInfo1.peer_addr, broadcast1, 6);
  peerInfo1.channel = 0;
  peerInfo1.encrypt = false;
  if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
    Serial.println("Failed to add peer 1");
    return;
  }

  memcpy(peerInfo2.peer_addr, broadcast2, 6);
  peerInfo2.channel = 0;
  peerInfo2.encrypt = false;
  if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
    Serial.println("Failed to add peer 2");
    return;
  }

  Wire.begin(21, 22);

  pinMode(online, OUTPUT);  //Set pinMode to OUTPUT for online LED

  wifi_Start();

  //FTP Setup, ensure SPIFFS is started before ftp;
  if (SPIFFS.begin(true))
  {
    Serial.println("SPIFFS opened!");
    Serial.println("");
    ftpSrv.begin(ftpUser, ftpPassword);    //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
  }
  //End FTP  

  /*
  serverAsync.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html);
  });
  */

  events.onConnect([](AsyncEventSourceClient * client) { 
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());

    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });

  serverAsync.addHandler(&events);   
  
  serverAsync.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    PATH = "/FAVICON";
    //accessLog();
    if (! flag == 1)
    {
      request->send(SPIFFS, "/favicon.ico", "image/ico");

    }
    //end();
  });

  serverAsync.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {

    PATH = "/";
    accessLog();

    ipREMOTE = request->client()->remoteIP();

    if (! flag == 1)
    {
      request->send_P(200, PSTR("text/html"), HTML1, processor1);
    }
    end();
  });

  serverAsync.on("/Weather", HTTP_GET, [](AsyncWebServerRequest * request)
  {

    PATH = "/Weather";
    accessLog();

    ipREMOTE = request->client()->remoteIP();

    if (! flag == 1)
    {
      request->send_P(200, PSTR("text/html"), HTML1, processor1);
    }
    end();
  });

  serverAsync.on("/SdBrowse", HTTP_GET, [](AsyncWebServerRequest * request)
  {

    PATH = "/SdBrowse";
    accessLog();

    if (! flag == 1)
    {
      request->send_P(200, PSTR("text/html"), HTML2, processor2);

    }
    end();
  });

  serverAsync.on("/Graphs", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    PATH = "/Graphs";
    accessLog();
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML3, processor3);
    response->addHeader("Server", "ESP Async Web Server");
    if (! flag == 1)
    {
      request->send(response);

    }
    end();
  });

  serverAsync.on("/get-file", HTTP_GET, [](AsyncWebServerRequest *request){
  PATH = fn;
  accessLog();
  if (! flag == 1)
  {

      request->send(SPIFFS, fn, "text/txt");

  }
  end();

  });  
  
  serverAsync.on("/Show", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    
    if (! flag == 1)
    {
      
      request->send_P(200, PSTR("text/html"), HTML6, processor6);
      
    }
    //end();
  });
  

  serverAsync.on("/RTSP", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    PATH = "/RTSP";
    accessLog();
    if (! flag == 1)
    {
      
      request->send_P(200, PSTR("text/html"), HTML7, processor7);
      
    }

    end();

    //switchRelay = 1;  // turn on camera

  });

  serverAsync.on("/Contactus", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    PATH = "/Contactus";
    accessLog();
    if (! flag == 1)
    {
      request->send_P(200, PSTR("text/html"), HTML5, processor5);

    }
    end();
  });

  /*
       serverAsync.on("/ACCESS.TXT", HTTP_GET, [](AsyncWebServerRequest * request)
       {
            PATH = "/ACCESS";
            accessLog();
            AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML3);
            response->addHeader("Server","ESP Async Web Server");
            if(! flag == 1)
            {
                 request->send(SPIFFS, "/ACCESS610.TXT");

            }
            end();
       });
  */

  serverAsync.on("/RESTART", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    PATH = "/RESTART";
    accessLog();
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML4, processor4);
    response->addHeader("Server", "ESP Async Web Server");
    if (! flag == 1)
    {
      request->send(response);

    }
    Serial2.flush();
    end();
    ESP.restart();

  });

  ///////////////////// OTA Support //////////////////////

  //attach.AsyncWebSocket
  ws.onEvent(onEvent);
  serverAsync.addHandler(&ws);

  // attach AsyncEventSource
  serverAsync.addHandler(&events);

  // respond to GET requests on URL /heap
  serverAsync.on("/heap", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  // upload a file to /upload
  serverAsync.on("/upload", HTTP_POST, [](AsyncWebServerRequest * request)
  {
    request->send(200);
  }, onUpload);

  // send a file when /index is requested
  serverAsync.on("/index", HTTP_ANY, [](AsyncWebServerRequest * request)
  {
    request->send(SPIFFS, "/index.htm");
  });

  // HTTP basic authentication
  serverAsync.on("/login", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    PATH = "/login";
    accessLog();
    if (!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send(200, "text/plain", "Login Success; upload firmware!");
    logon = 1;
    end();
  });

  // Simple Firmware Update Form
  serverAsync.on("/update", HTTP_GET, [](AsyncWebServerRequest * request)
  {

    PATH = "/update";
    accessLog();
    if (logon == 1)
    {
      request->send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
      logon = 0;
      end();
    }
    else
    {
      request->send(404); //Sends 404 File Not Found
      logon = 0;
      end();
    }


  });

  serverAsync.on("/update", HTTP_POST, [](AsyncWebServerRequest * request)
  {
    shouldReboot = !Update.hasError();

    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
  }, [](AsyncWebServerRequest * request, String filename, size_t index, uint8_t *data, size_t len, bool final)
  {

    if (!index)
    {
      Serial.printf("Update Start: %s\n", filename.c_str());
      //Update.runAsync(true);
      if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000))
      {
        Update.printError(Serial);
      }
    }
    if (!Update.hasError())
    {
      if (Update.write(data, len) != len)
      {
        Update.printError(Serial);
        end();
      }
    }
    if (final)
    {
      if (Update.end(true))
      {
        Serial.printf("Update Success: %uB\n", index + len);
        end();
      }
      else
      {
        Update.printError(Serial);
      }
    }


  });


  // attach filesystem root at URL /fs
  //serverAsync.serveStatic("/fs", SPIFFS, "/");

  // Catch-All Handlers
  // Any request that can not find a Handler that canHandle it
  // ends in the callbacks below.
  serverAsync.onNotFound(onRequest);
  serverAsync.onFileUpload(onUpload);
  serverAsync.onRequestBody(onBody);

  //serverAsync.begin();

  ///////////////////////// End OTA Support /////////////////////////////

  serverAsync.onNotFound(notFound);

  secondTick.attach(1, ISRwatchdog);  //watchdog  ISR triggers every 1 second

  configTime(0, 0, udpAddress1, udpAddress2);
  setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 3);   // this sets TZ to Indianapolis, Indiana
  tzset();

  /*
       Serial.print("wait for first valid timestamp ");

       while (time(nullptr) < 100000ul)
       {
            Serial.print(".");
            delay(5000);
       }
  */

  pinMode(REEDPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REEDPIN), reedSwitch_ISR, FALLING);

  // initialize EEPROM with predefined size
  EEPROM.begin(eeprom_size);

  //RESET EEPROM CONTENT - ONLY EXECUTE ONE TIME - AFTER COMMENT

  

       // Uncomment to 'clear'.eeprom values.
       /*
       Serial.println("CLEAR ");
       eepromClear();
       Serial.println("SET ");
       eepromSet("rain5min", "0.00");
       eepromSet("rainDay", "0.00");
       eepromSet("rainHour", "0.00");
       Serial.println("LIST ");
       Serial.println(eepromList());
       */

  //END - RESET EEPROM CONTENT - ONLY EXECUTE ONE TIME - AFTER COMMENT
  //eepromClear();

  //GET STORED RAINCOUNT IN EEPROM
  Serial.println("");
  Serial.println("GET EEPROM --Setup");
  eepromstring = eepromGet("rainDay");
  rainDay = eepromstring.toFloat();
  Serial.print("RAINDAY VALUE FROM EEPROM: ");
  Serial.println(rainDay);

  eepromstring = eepromGet("rainHour");
  rainHour = eepromstring.toFloat();
  Serial.print("RAINHOUR VALUE FROM EEPROM: ");
  Serial.println(rainHour);

  eepromstring = eepromGet("rain5min");
  rain5min = eepromstring.toFloat();
  Serial.print("rain5min VALUE FROM EEPROM: ");
  Serial.println(rain5min);
  Serial.println("");
  //END - GET STORED RAINCOUNT IN EEPROM

  //SPIFFS.format();

  //lcdDisplay();      //   LCD 1602 Display function --used for inital display

  ThingSpeak.begin(client);

  //delay(30 * 1000);  //Used to test reconnect WiFi routine.  Will produce one entry for each disconnect in "WIFI.TXT."

  //WiFi.disconnect();  //Used to test reconnect WiFi routine.  Will produce one entry for eac disconnect in "WIFI.TXT."

  //delay(50 * 1000);  //Uncomment to test watchdog

  //Serial.println("Delay elapsed");

  started = 1;
  watchdogCounter = 0;
  powerOn = 1;

}

void loop() 
{
  /* Updates frequently */
  unsigned long currentTime = millis();

  if(cameraPoweroff == 1){
    Serial.println("Camera Power turned off");
    cameraPoweroff = 0;
    cameraCounter = 0;
  }

  //udp only send data when connected
  if (connected)
  {

    //Send a packet
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("Seconds since boot: %u", millis() / 1000);
    udp.endPacket();
  }
        
  digitalWrite(online, LOW);

  delay(1);

  if ((brownout == 1) || (softReset == 1))
  {

    watchdogCounter = 0;  //Resets the watchdogCounter

    getDateTime();

    //Open a "WIFI.TXT" for appended writing.   Client access ip address logged.
    File logFile = SPIFFS.open("/WIFI.TXT", "a");

    if (!logFile)
    {
      Serial.println("File: '/WIFI.TXT' failed to open");
    }

    if ((brownout == 1) && (powerOn != 1))
    {

      logFile.println("ESP32 Restart caused by Brownout Detection...");

      brownout = 0;

      connect = 1;

    }

    if ((brownout == 1) && (powerOn == 1))
    {

      powerOn = 0;

      Serial.println("ESP32 Webserver Started...");

      logFile.println("ESP32 Webserver Started...");

      brownout = 0;

      connect = 1;

    }

    if (softReset == 1)
    {

      logFile.println("ESP32 Restart caused by Watchdog Event...");      

    }

    softReset = 0;   

    logFile.close();    

  }

  if (connect == 1)
  {

    //Serial.println("Connect:  " + (String)connect);

    configTime();
    
    delay(1000);

    //Open a "WIFI.TXT" for appended writing.
    File logFile = SPIFFS.open("/WIFI.TXT", "a");

    if (!logFile)
    {
      Serial.println("File: '/WIFI.TXT' failed to open");
    }

    logFile.print("WiFi Connected:       ");

    logFile.print(dtStamp);

    logFile.printf("   Connection result: %d\n", WiFi.waitForConnectResult());

    logFile.println("");

    logFile.close();

    Serial.println("Logged WiFi connection, timestamp\n");

    Serial.println("Ready...\n");

    watchdogCounter = 0;  //Resets the watchdogCounter

    connect = 0;

    counter = 1;

  }

  if (WiFi.status() != WL_CONNECTED) 
  {

    watchdogCounter = 0;  //Resets the watchdogCounter

    //Open "WIFI.TXT" for appended writing.   Client access ip address logged.
    File logFile = SPIFFS.open("/WIFI.TXT", "a");

    if (!logFile)
    {
      Serial.println("File: '/WIFI.TXT' failed to open");
    }

    getDateTime();

    logFile.print("WiFi Disconnected:   ");

    logFile.print(dtStamp);

    logFile.print("   Connection result: ");

    logFile.println(WiFi.waitForConnectResult());

    logFile.println("");

    logFile.close();

    Serial.println("\nLogged WiFi disconnection, timestamp\n");

    watchdogCounter = 0;

    wifi_Start();

    configTime();

    disconnect = 0;
    counter = 0;
    connect = 1;

  }

  if (watchDog == 1)
  {

    portENTER_CRITICAL(&mux);
    watchdogCounter--;
    portEXIT_CRITICAL(&mux);

    getDateTime();

    logWatchdog();

  }

  watchdogCounter = 0;  //Resets the watchdogCount

  ///////////////////////////////////////////////////// FTP ///////////////////////////////////
  for (int x = 1; x < 5000; x++)
  {
    ftpSrv.handleFTP();
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////// OTA Support ///////////////////////

  if (shouldReboot)
  {
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
  }

  static char temp[128];
  sprintf(temp, "Seconds since boot: %u", (currentTime - previousTime_2));   // / 1000);   //time since boot
  events.send(temp, "time"); //send event "time"

  previousTime_2 = currentTime;

  //////////////////// End OTA Support /////////////////////////

  // each second read and reset pulseCount_ISR
  //if (millis() - lastSecond >= 1000)
  if ( currentTime - previousTime_3 >= eventTime_3) {   //rain gauge interupt

    lastSecond += 1000;
    portENTER_CRITICAL(&mux);
    currentPulseCount += pulseCount_ISR; // add to current counter
    pulseCount_ISR = 0; // reset ISR counter
    rainFall = currentPulseCount * .047; //Amout of rain in one bucket dump.
    portEXIT_CRITICAL(&mux);

    previousTime_3 = currentTime;

  }



  // each 5 minutes save data to another counter
  //if (millis()-last5Minutes>=FIVEMINUTES)
  if (currentTime - previousTime_4 >= eventTime_4) {   //rain gauge

    rain5min = rainFall;
    rainHour = rainHour + rainFall;  //accumulaing 5 minute rainfall for 1 hour then reset -->rainHour Rainfall
    rainDay = rainDay + rainFall;  //aacumulating 1 day rainfall
    last5Minutes += FIVEMINUTES; // remember the time
    lastPulseCount += currentPulseCount; // add to last period Counter
    currentPulseCount = 0;; // reset counter for current period

    previousTime_4 = currentTime;

  }

  getDateTime();

  //Serial.println(dtStamp);

  //Executes every 1 Minute Routine.
  if ((MINUTE % 1 == 0) && (SECOND == 0))
  {

    delay(2000);

    static unsigned long lastEventTime = millis();
    static const unsigned long EVENT_INTERVAL_MS = 60 * 1000;  //20000 = 20 seconds
    //if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    if (currentTime - previousTime_5 >= eventTime_5) {   //sender

      events.send("ping", NULL, millis());
      lastEventTime = millis();
      previousTime_5 = currentTime;

    }
  }

  //Executes 5 Minute Routine.
  if ((MINUTE % 5 == 0) && (SECOND == 0))
  {

    delay(2000);

    Serial.println("");
    Serial.println("Five Minute routine");
    Serial.println(dtStamp);



    //STORE RAINCOUNT IN EEPROM
    Serial.println("SET EEPROM rainHour");
    eepromstring = String(rainHour, 2);
    eepromSet("rainHour", eepromstring);
    //END - STORE RAINCOUNT IN EEPROM

    //STORE RAINCOUNT IN EEPROM
    Serial.println("SET EEPROM rainDay");
    eepromstring = String(rainDay, 2);
    eepromSet("rainDay", eepromstring);
    //END - STORE RAINCOUNT IN EEPROM

    //STORE RAINCOUNT IN EEPROM
    Serial.println("SET EEPROM rain5min");
    eepromstring = String(rain5min, 2);
    eepromSet("rain5min", eepromstring);
    //END - STORE RAINCOUNT IN EEPROM
    Serial.println("");

    rainFall = 0;
    rain5min = 0;

    flag = 0;

  }

  //Executes 15 Minute routine and one Five Minute Rountine.
  if ((MINUTE % 15 == 0) && (SECOND == 0))
  {

    Serial.println("");
    Serial.println("Fifthteen minute routine");
    Serial.println(dtStamp);

    delayTime = 1000;

    lastUpdate = dtStamp;   //store dtstamp for use on dynamic web page
    updateDifference();  //Get Barometric Pressure difference
    logtoSD();   //Output to LittleFS  --Log to LittleFS on 15 minute interval.
    delay(10);  //Be sure there is enough LittleFS write time
    pastPressure = currentPressure;  //Store last 15 MINUTE, currentPressure
    //webInterface();
    //speak();


  }

  if ((MINUTE == 59) && (SECOND == 59)) // one hour counter
  {
    rainHour = 0;
    rain5min = 0;
    rainFall = 0;
  }
  
  if ((HOUR == 23) && (MINUTE == 58) && (SECOND == 58)) //24 Hour; clear ALL.
  {
    fileStore();
  
    rain5min = 0;
    rainFall = 0;
    rainHour = 0;
    rainDay = 0;
    daysRain = 0;
  
  }
  
}

String processor1(const String& var)
{

  //index1.h

  if (var == F("LASTUPDATE"))
    return lastUpdate;

  if (var == F("GPSLAT"))
    return String(gpslat, 5);

  if (var == F("GPSLNG"))
    return String(gpslng, 5);

  if (var == F("GPSALT"))
    return String(gpsalt, 1);

  if (var == F("TEMP"))
    return String(myData.temp, 2);

  if (var == F("HEATINDEX"))
    return String(myData.heat, 2);

  if (var == F("HUM"))
    return String(myData.hum, 2);

  if (var == F("DEWPOINT"))
    return String(myData.dew, 2);

  if (var == F("PRESSURE"))
    return String(myData.press, 3);

  if (var == F("DIF"))
    return String((difference), 3);


  if (var == F("RAINDAY"))
    return String(rainDay);

  if (var == F("RAINHOUR"))
    return String(rainHour);

  if (var == F("RAINFALL"))
    return String(rainFall);

  if (var == F("DTSTAMP"))
    return dtStamp;

  if (var == F("LINK"))
    return linkAddress;

  if (var == F("CLIENTIP"))
    return ipREMOTE.toString().c_str();

  return String();

}

String processor2(const String& var)
{

  //index2.h

  String str;

  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS failed to mount !");
  }


  File root = SPIFFS.open("/");

  File file = root.openNextFile();

  while (file)
  {

    String file_name = file.name();

    if (file_name.startsWith("LOG"))
    {
      str += "<a href=\"";
      str += file.name();
      str += "\">";
      str += file.name();
      str += "</a>";
      str += "    ";
      str += file.size();
      str += "<br>\r\n";

    }

    file = root.openNextFile();
  }

  root.close();

  //root.rewindDirectory();

  if (var == F("URLLINK"))
    return  str;

  if (var == F("LINK"))
    return linkAddress;

  if (var == F("FILENAME"))
    return  file.name();

  return String();

}

String processor3(const String& var)
{

  //index3.h

  if (var == F("LINK"))
    return linkAddress;

  return String();

}

String processor4(const String& var)
{

  //index4.h

  if (var == F("LINK"))
    return linkAddress;

  return String();

}

String processor5(const String& var)
{

  //index5.h

  if (var == F("LINK"))
    return linkAddress;

  return String();

}

String processor6(const String& var)
{

  //index6.h

  if (var == F("FN"))
    return fn;

  if (var == F("LINK"))
    return linkAddress;

  return String();

}

String processor7(const String& var)
{

  //index7.h

  if (var == F("LINK"))
    return linkAddress;

  return String();

}

///////////////////////////////////////

void accessLog()
{

  digitalWrite(online, HIGH);  //turn on online LED indicator

  getDateTime();

  String ip1String = "10.0.0.146";   //Host ip address
  String ip2String = ipREMOTE.toString().c_str();   //client remote IP address

  Serial.println("");
  Serial.println("Client connected:  " + dtStamp);
  Serial.print("Client IP:  ");
  Serial.println(ip2String);
  Serial.print("Path:  ");
  Serial.println(PATH);
  Serial.println(F("Processing request"));

  //Open a "access.txt" for appended writing.   Client access ip address logged.
  File logFile = SPIFFS.open(Restricted, FILE_APPEND);

  if (!logFile)
  {
    Serial.println("File 'ACCESS.TXT'failed to open");
  }

  if ((ip1String == ip2String) || (ip2String == "0.0.0.0") || (ip2String == "(IP unset)"))
  {

    //Serial.println("HOST IP Address match");
    logFile.close();

  }
  else
  {

    Serial.println("Log client ip address");

    logFile.print("Accessed:  ");
    logFile.print(dtStamp);
    logFile.print(" -- Client IP:  ");
    logFile.print(ip2String);
    logFile.print(" -- ");
    logFile.print("Path:  ");
    logFile.print(PATH);
    logFile.println("");
    logFile.close();

  }

}

void beep(unsigned char delayms)
{

  //     wait for a delayms ms
  //     digitalWrite(sonalert, HIGH);
  //     delayTime = 3000;
  //     digitalWrite(sonalert, LOW);

}

void configTime()
{

  configTime(0, 0, udpAddress1, udpAddress2);
  setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 3);   // this sets TZ to Indianapolis, Indiana
  tzset();

  //udp only send data when connected
  if (connected)
  {

    //Send a packet
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("Seconds since boot: %u", millis() / 1000);
    udp.endPacket();
  }

  Serial.print("wait for first valid timestamp");

  while (time(nullptr) < 100000ul)
  {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("\nSystem Time set\n");

  getDateTime();

  Serial.println(dtStamp);

}

//--- EEPROM -----coded by----- Muhammad Haroon 

void eepromSet(String name, String value)
{
  Serial.println("eepromSet");

  String list = eepromDelete(name);
  String nameValue = "&" + name + "=" + value;
  //Serial.println(list);
  //Serial.println(nameValue);
  list += nameValue;
  for (int i = 0; i < list.length(); ++i)
  {
    EEPROM.write(i, list.charAt(i));
  }
  EEPROM.commit();
  Serial.print(name);
  Serial.print(":");
  Serial.println(value);

  delayTime = 1000;

}


String eepromDelete(String name)
{
  Serial.println("eepromDelete");

  int nameOfValue;
  String currentName = "";
  String currentValue = "";
  int foundIt = 0;
  char letter;
  String newList = "";
  for (int i = 0; i < 512; ++i)
  {
    letter = char(EEPROM.read(i));
    if (letter == '\n')
    {
      if (foundIt == 1)
      {
      }
      else if (newList.length() > 0)
      {
        newList += "=" + currentValue;
      }
      break;
    }
    else if (letter == '&')
    {
      nameOfValue = 0;
      currentName = "";
      if (foundIt == 1)
      {
        foundIt = 0;
      }
      else if (newList.length() > 0)
      {
        newList += "=" + currentValue;
      }

    }
    else if (letter == '=')
    {
      if (currentName == name)
      {
        foundIt = 1;
      }
      else
      {
        foundIt = 0;
        newList += "&" + currentName;
      }
      nameOfValue = 1;
      currentValue = "";
    }
    else
    {
      if (nameOfValue == 0)
      {
        currentName += letter;
      }
      else
      {
        currentValue += letter;
      }
    }
  }

  for (int i = 0; i < 512; ++i)
  {
    EEPROM.write(i, '\n');
  }
  EEPROM.commit();
  for (int i = 0; i < newList.length(); ++i)
  {
    EEPROM.write(i, newList.charAt(i));
  }
  EEPROM.commit();
  Serial.println(name);
  Serial.println(newList);
  return newList;
}

void eepromClear()
{
  Serial.println("eepromClear");
  for (int i = 0; i < 512; ++i)
  {
    EEPROM.write(i, '\n');
  }
}

String eepromList()
{
  Serial.println("eepromList");
  char letter;
  String list = "";
  for (int i = 0; i < 512; ++i)
  {
    letter = char(EEPROM.read(i));
    if (letter == '\n')
    {
      break;
    }
    else
    {
      list += letter;
    }
  }
  Serial.println(list);
  return list;
}

String eepromGet(String name)
{
  Serial.println("eepromGet");

  int nameOfValue;
  String currentName = "";
  String currentValue = "";
  int foundIt = 0;
  String value = "";
  char letter;
  for (int i = 0; i < 512; ++i)
  {
    letter = char(EEPROM.read(i));
    if (letter == '\n')
    {
      if (foundIt == 1)
      {
        value = currentValue;
      }
      break;
    }
    else if (letter == '&')
    {
      nameOfValue = 0;
      currentName = "";
      if (foundIt == 1)
      {
        value = currentValue;
        break;
      }
    }
    else if (letter == '=')
    {
      if (currentName == name)
      {
        foundIt = 1;
      }
      else
      {
      }
      nameOfValue = 1;
      currentValue = "";
    }
    else
    {
      if (nameOfValue == 0)
      {
        currentName += letter;
      }
      else
      {
        currentValue += letter;
      }
    }
  }
  Serial.print(name);
  Serial.print(":");
  Serial.println(value);
  return value;
}

void seteeprom()
{

  eepromstring = String(rainDay, 2);
  eepromSet("rainDay", eepromstring);

  rain5min = 0;

  eepromstring = String(rainHour, 2);
  eepromSet("rainHour", eepromstring);

  eepromstring = String(rain5min, 2);
  eepromSet("rain5min", eepromstring);


  //END - STORE RAINCOUNT IN EEPROM

}

//end EEPROM ---coded by --- Muhammad Haroon 

void end()
{

  delay(1000);

  digitalWrite(online, LOW);   //turn-off online LED indicator

  getDateTime();

  Serial.println("Client closed:  " + dtStamp);

}

void fileStore()   //If Midnight, rename "LOGXXYYZZ.TXT" to ("log" + month + day + ".txt") and create new, empty "LOGXXYYZZ.TXT"
{

  int temp;
  String Date;
  String Month;

  temp = (DATE);
  if (temp < 10)
  {
    Date = ("0" + (String)temp);
  }
  else
  {
    Date = (String)temp;
  }

  temp = (MONTH);
  if (temp < 10)
  {
    Month = ("0" + (String)temp);
  }
  else
  {
    Month = (String)temp;
  }

  String logname;  //file format /LOGxxyyzzzz.txt
  logname = "/LOG";
  logname += Month;   ///logname += Clock.getDate();
  logname += Date; ////logname += Clock.getMonth(Century);
  logname += YEAR;
  logname += ".TXT";

  //Open file for appended writing
  File log = SPIFFS.open(logname.c_str(), "a");

  if (!log)
  {
    Serial.println("file open failed");
  }

}


//Get Date and Time
String getDateTime()
{
  struct tm *ti;

  tnow = time(nullptr) + 1;
  //strftime(strftime_buf, sizeof(strftime_buf), "%c", localtime(&tnow));
  ti = localtime(&tnow);
  DOW = ti->tm_wday;
  YEAR = ti->tm_year + 1900;
  MONTH = ti->tm_mon + 1;
  DATE = ti->tm_mday;
  HOUR  = ti->tm_hour;
  MINUTE  = ti->tm_min;
  SECOND = ti->tm_sec;

  strftime(strftime_buf, sizeof(strftime_buf), "%a , %m/%d/%Y , %H:%M:%S %Z", localtime(&tnow));
  dtStamp = strftime_buf;
  return (dtStamp);

}


//Pressure difference for fifthteen minute interval
float updateDifference()  //Pressure difference for fifthteen minute interval
{


  //Function to find difference in Barometric Pressure
  //First loop pass pastPressure and currentPressure are equal resulting in an incorrect difference result.  Output "...Processing"
  //Future loop passes difference results are correct

  difference = currentPressure - pastPressure;  //This will be pressure from this pass thru loop, pressure1 will be new pressure reading next loop pass
  if (difference == currentPressure)
  {
    difference = 0;
  }
  return (difference); //Barometric pressure change in inches of Mecury

}

//Write to SPIFFS
void logtoSD()   //Output to SPIFFS every fifthteen minutes
{

  getDateTime();

  int tempy;
  String Date;
  String Month;

  tempy = (DATE);
  if (tempy < 10)
  {
    Date = ("0" + (String)tempy);
  }
  else
  {
    Date = (String)tempy;
  }

  tempy = (MONTH);
  if (tempy < 10)
  {
    Month = ("0" + (String)tempy);
  }
  else
  {
    Month = (String)tempy;
  }

  String logname;
  logname = "/LOG";
  logname += Month;   ///logname += Clock.getDate();
  logname += Date; ////logname += Clock.getMonth(Century);
  logname += YEAR;
  logname += ".TXT";

  // Open a "log.txt" for appended writing
  //File log = SPIFFS.open(logname.c_str(), FILE_APPEND);
  File log = SPIFFS.open(logname.c_str(), FILE_APPEND);

  if (!log)
  {
    Serial.println("file 'LOG.TXT' open failed");
  }

  delay(500);

  //log.print(id);
  //log.print(" , ");
  log.print("Lat: ");
  log.print(gpslat, 5);
  log.print(" , ");
  log.print("Long: ");
  log.print(gpslng, 5);
  log.print(" , ");
  log.print(lastUpdate);
  log.print(" , ");
  
  log.print("Temp:  ");
  log.print(temperature, 1);
  log.print(" F. , ");
  
  log.print("HeatId:  ");
  log.print(heatId, 1);
  log.print(" F. , ");

  log.print("Hum:  ");
  log.print(humd, 1);
  log.print(" % , ");
  
  log.print("DewPt:  ");
  log.print(dewPt, 1);
  log.print(" % , ");  

  log.print("Press:  ");
  log.print(currentPressure, 3);
  log.print(" inHg. ");
  log.print(" , ");

  if (pastPressure == currentPressure)
  {
    log.print("0.000");
    log.print(" Difference ");
    log.print(" ,");
  }
  else
  {
    log.print((difference), 3);
    log.print(" Difference ");
    log.print(", ");
  }

  log.print(" Day ");
  log.print(rainDay, 2);
  log.print(" ,");

  log.print(" Hour ");
  log.print(rainHour, 2);
  log.print(" , ");

  log.print(" Five Minute ");
  log.print(rain5min, 2);
  log.print(" , ");

  log.print("Elev:  ");
  log.print(gpsalt, 0);
  log.print(" feet. ");
  log.println();

  //Increment Record ID number
  //id++;

  Serial.println("");

  Serial.println("Data written to  " + logname + "  " + dtStamp);

  log.close();

  pastPressure = currentPressure;

  if (abs(difference) >= .020) //After testing and observations of Data; raised from .010 to .020 inches of Mecury
  {
    // Open a "Differ.txt" for appended writing --records Barometric Pressure change difference and time stamps
    File diffFile = SPIFFS.open("DIFFER.TXT", FILE_APPEND);

    if (!diffFile)
    {
      Serial.println("file 'DIFFER.TXT' open failed");
    }

    Serial.println("");
    Serial.print("Difference greater than .020 inches of Mecury ,  ");
    Serial.print(difference, 3);
    Serial.print("  ,");
    Serial.print(dtStamp);

    diffFile.println("");
    diffFile.print("Difference greater than .020 inches of Mecury,  ");
    diffFile.print(difference, 3);
    diffFile.print("  ,");
    diffFile.print(dtStamp);
    diffFile.close();

    beep(50);  //Duration of Sonalert tone

  }
}

void logWatchdog()
{

  //yield();

  Serial.println("");
  Serial.println("Watchdog event triggered.");

  Serial.println("Watchdog Restart  " + dtStamp);

  ESP.restart();

}

//readFile  --AsyncWebServer version with much help from Pavel
String notFound(AsyncWebServerRequest *request)
{

  digitalWrite(online, HIGH);   //turn-on online LED indicator

  if (! request->url().endsWith(F(".TXT")))
  {
    request->send(404);
  }
  else
  {
    if (request->url().endsWith(F(".TXT")))
    {
      //.endsWith(F(".txt")))

      // here comes some mambo-jambo to extract the filename from request->url()
      int fnsstart = request->url().lastIndexOf('/');

      fn = request->url().substring(fnsstart);

      uncfn = fn.substring(1);

      urlPath = linkAddress + "/" + uncfn;            

    }    

  }
  
  request->redirect("/Show");

  digitalWrite(online, LOW);   //turn-off online LED indicator 

  return fn; 

}


void printincomingReadings()
{
  Serial.println(temperature);
  Serial.println(heatId);
  Serial.println(humd);
  Serial.println(dewPt);
  Serial.println(currentPressure);
  Serial.println("");
}

void readings1()
{

  temperature = myData.temp;
  heatId = myData.heat;
  humd = myData.hum;
  dewPt = myData.dew;
  currentPressure = myData.press;
}

void readings2()
{

  controlRelay = relayReadings.ctrl;
  relayStatus = relayReadings.stat;
  readingId = relayReadings.readingId++;
}

//ThingSpeak.com --Graphing and iftmes
void speak()
{

  char t_buffered1[14];
  dtostrf(temperature, 7, 1, t_buffered1);

  char t_buffered2[14];
  dtostrf(humd, 7, 1, t_buffered2);

  char t_buffered3[14];
  dtostrf(currentPressure, 7, 1, t_buffered3);

  char t_buffered4[14];
  dtostrf(dewPt, 7, 1, t_buffered4);

  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  Here, we write to field 1.
  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  Here, we write to field 1.
  ThingSpeak.setField(1, t_buffered1);  //Temperature
  ThingSpeak.setField(2, t_buffered2);  //Humidity
  ThingSpeak.setField(3, t_buffered3);  //Barometric Pressure
  ThingSpeak.setField(4, t_buffered4);  //Dew Point F.

  // Write the fields that you've set all at once.
  //ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  getDateTime();

  Serial.println("");
  Serial.println("Sent data to Thingspeak.com  " + dtStamp + "\n");

}

//Hosted Domain, web page -code sends data for Dynamic web page every 15 Minutes
void webInterface()
{

  char glat[10]; // Buffer big enough for 9-character float
  dtostrf(gpslat, 9, 4, glat); // Leave room for too large numbers!

  char glng[10]; // Buffer big enough for 9-character float
  dtostrf(gpslng, 9, 4, glng); // Leave room for too large numbers!

  char fahr[7];// Buffer big enough for 9-character float
  dtostrf(temperature, 6, 1, fahr); // Leave room for too large numbers!

  char heatindex[7];// Buffer big enough for 9-character float
  dtostrf(heatId, 6, 1, heatindex); // Leave room for too large numbers!

  char humidity[7]; // Buffer big enough for 9-character float
  dtostrf(humd, 6, 1, humidity); // Leave room for too large numbers!

  char dewpoint[7]; // Buffer big enough for 9-character float
  dtostrf(dewPt, 6, 1, dewpoint); // Leave room for too large numbers!

  char barometric[9]; // Buffer big enough for 7-character float
  dtostrf(currentPressure, 8, 3, barometric); // Leave room for too large numbers!

  char diff[9]; // Buffer big enough for 7-character float
  dtostrf(difference, 8, 3, diff); // Leave room for too large numbers!

  char rain5[10]; // Buffer big enough for 9-character float
  dtostrf(rain5min, 6, 3, rain5); // Leave room for too large numbers!

  char rain60[10]; // Buffer big enough for 9-character float
  dtostrf(rainHour, 6, 3, rain60); // Leave room for too large numbers!

  char rain24[10]; // Buffer big enough for 9-character float
  dtostrf(rainDay, 6, 3, rain24); // Leave room for too large numbers!

  char alt[9]; // Buffer big enough for 9-character float
  dtostrf(843, 8, 1, alt); // Leave room for too large numbers!

  String data = "&last="                  +  (String)lastUpdate

                + "&glat="                +  glat

                + "&glng="                +  glng

                + "&fahr="                +  fahr

                + "&heatindex="           +  heatindex

                + "&humidity="            +  humidity

                + "&dewpoint="            +  dewpoint

                + "&barometric="          +  barometric

                + "&diff="                +  diff

                + "&rain5="               +  rain5

                + "&rain60="              +  rain60

                + "&rain24="              +  rain24

                + "&alt="                 +  alt;

  if (WiFi.status() == WL_CONNECTED)
  {
    //Check WiFi connection status

    HTTPClient http;    //Declare object of class HTTPClient

    http.begin(sendData);      //Specify request destination
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");  //Specify content-type header

    int httpCode = http.POST(data);   //Send the request
    String payload = http.getString();                  //Get the response payload

    if (httpCode == 200)
    {
      Serial.print("");
      Serial.print("HttpCode:  ");
      Serial.print(httpCode);   //Print HTTP return code
      Serial.print("  Data echoed back from Hosted website  " );
      Serial.println("");
      Serial.println(payload);    //Print payload response

      http.end();  //Close HTTPClient http
    }
    else
    {
      Serial.print("");
      Serial.print("HttpCode:  ");
      Serial.print(httpCode);   //Print HTTP return code
      Serial.print("  Domain website data update failed.  ");
      Serial.println("");

      http.end();   //Close HTTPClient http
    }

  }
  else
  {

    Serial.println("Error in WiFi connection");

  }

}

//WiFi Connect
void wifi_Start()
{

  //WiFi.disconnect();

  //delayTime = (10 * 1000);

  //WiFi.mode(WIFI_OFF);

  WiFi.mode(WIFI_AP_STA);

  Serial.println();
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  // We start by connecting to WiFi Station
  Serial.print("Connecting to ");
  Serial.println(ssidStation);

  WiFi.begin(ssidStation, passwordStation);
  delay(1000);

  //setting the static addresses in function "wifi_Start
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;

  WiFi.config(ip, gateway, subnet, dns);

  WiFi.begin(ssidStation, passwordStation);


  Serial.println("Web server running. Waiting for the ESP32 IP...");

  // Printing the ESP IP address
  Serial.print("Server IP:  ");
  Serial.println(WiFi.localIP());
  Serial.print("Port:  ");
  Serial.println(LISTEN_PORT);
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
  Serial.println("\n");

  delayTime = 500;

  WiFi.waitForConnectResult();

  serverAsync.begin();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.printf("Connection result: %d\n", WiFi.waitForConnectResult());

  serverAsync.begin();

  if (WiFi.waitForConnectResult() != 3)
  {
    delay(3000);
    wifi_Start();
  }
}

