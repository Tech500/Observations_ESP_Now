/*
  William Lucid and Google's Bard
  ESP_Now__Relay_board_3.ino   Relay control and Relay status  
*/

#include <WiFi.h>
#include <esp_now.h>

#define Board_Id 3
//Board_3 mac address:  30:83:98:D9:1D:EC    ESP Pico D4

#define CHANNEL 10

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x3C, 0xE9, 0x0E, 0x85, 0x27, 0x40};   //Board_2 mac address  ESP32 Node32s

// Define variables to store incoming readings
int controlRelay;  //Open relay contact
int relayStatus;  //Status , relay is open

// Updates DHT readings every 10 seconds
const long interval = 30 * 1000; 
unsigned long previousMillis = 0;    // will store last time DHT was updated 

// Variable to store if sending data was successful
String success;

// Board_2 and Board_3 
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id;
    int ctrl;
    int stat;
    int readingId;
} struct_message;

// Create a struct_message called relayReadings to hold relay readings
struct_message incomingReadings2;


// Create a struct_message to hold incoming relay readings  //Relay control commands
struct_message relayReadings;

esp_now_peer_info_t peerInfo;

#define relayPin 25

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings2, incomingData, sizeof(incomingReadings2));
  Serial.print("Bytes received: ");
  Serial.println(len);
  controlRelay = incomingReadings2.ctrl;
  relayControl();
}

void printIncomingReadings(){
  // Display Readings in Serial Monitor
  Serial.println("INCOMING READINGS");
  Serial.print("Relay Control: ");
  Serial.println(controlRelay);
  if(controlRelay == 0){
    Serial.println("Relay contact is open.");
  }else{
    Serial.println("Relay contact is closed");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT);

  digitalWrite(relayPin, LOW);  //Relay contact is open
 
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
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  //peerInfo.channel = 10;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    relayControl();

    //Set values to send
    relayReadings.stat = relayStatus;
    
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &relayReadings, sizeof(relayReadings));


    // Print incoming readings
    //printIncomingReadings();
  }
}

void relayControl(){
  if (controlRelay == 0){
    controlRelay = 0;
    digitalWrite(relayPin, LOW);
    relayStatus = digitalRead(relayPin);
    printIncomingReadings();
  }
  else{
    controlRelay = 1;
    digitalWrite(relayPin, HIGH);
    relayStatus = digitalRead(relayPin);
    printIncomingReadings();
  }
}