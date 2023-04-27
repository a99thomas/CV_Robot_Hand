/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and multiple ESP32 Slaves
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave(s)

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/



#include <esp_now.h>
#include <WiFi.h>
#include <Servo_ESP32.h>

#define CHANNEL 1
#define LED 33


int angleNum;
int angleVal;

Servo_ESP32 servo1;  // create servo object to control a servo
Servo_ESP32 servo2;
Servo_ESP32 servo3;
// Servo_ESP32 servo4;
Servo_ESP32 servo5;
Servo_ESP32 servo6;
// Servo_ESP32 servo7;
Servo_ESP32 servo8;
// Servo_ESP32 servo9;
Servo_ESP32 servo10;
// twelve servo objects can be created on most boards


typedef struct struct_message {
  int a;
  uint8_t b;
} struct_message;
struct_message myData;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL ");
    Serial.println(WiFi.channel());
  }
}

void setup() {
  Serial.begin(115200);
  servo1.attach(18);  //Thumb Knuckle // attaches the servo on pin 9 to the servo object
  servo2.attach(19);  //Index Knuckle
  servo3.attach(27);  //Middle Knuckle
  // servo4.attach(8); //Ring Knuckle
  servo5.attach(5);   //Pinky Knuckle
  servo6.attach(16);  // Thumb Tilt
  // servo7.attach(4); //Index Tilt (not sure)
  servo8.attach(0);  //Middle Tilt
  // servo9.attach(2);//Ring Tilt
  servo10.attach(15);  //Pinky Tilt

  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  //LED Setup
  pinMode(LED, OUTPUT);
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  memcpy(&myData, data, sizeof(myData));
  //snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  //Serial.print("Last Packet Recv Data: ");
  Serial.print(myData.a);
  Serial.print(", ");
  Serial.println(myData.b);
  // servoSet(myData.a, myData.b);
  angleNum = myData.a;
  angleVal = myData.b;
}

void loop() {
  if (angleNum == 1) {
    int angleVal1 = map(angleVal, 0, 90, 110, 0);
    servo1.write(angleVal1);

    int angleVal6 = map(angleVal, 0, 90, 10, 120);
    servo6.write(angleVal6);
  }
  if (angleNum == 2) {
    int angleVal2 = map(angleVal, 0, 130, 10, 180);
    servo2.write(angleVal2);
  }
  if (angleNum == 3) {
    int angleVal3 = map(angleVal, 0, 180, 0, 180);
    servo3.write(angleVal3);
  }
  // if (angleNum == 4){
  //   servo4.write(angleVal);
  // }
  if (angleNum == 5) {
    int angleVal5 = map(angleVal, 0, 130, 0, 170);
    servo5.write(angleVal5);
  }
  // if (angleNum == 1) {
  //   int angleVal6 = map(angleVal, 0, 90, 10, 120);
  //   servo6.write(angleVal6);
  // }
  // if (angleNum == 7){
  //   servo7.write(angleVal);
  // }
  if (angleNum == 8) {
    int angleVal8 = map(angleVal, 0, 20, 60, 80);
    servo8.write(angleVal8);
  }
  // if (angleNum == 9){
  //   servo9.write(angleVal);
  // }
  if (angleNum == 10) {
    int angleVal10 = map(angleVal, 0, 40, 112, 130);
    servo10.write(angleVal10);
  }
//delay(10);
  //Chill
}
