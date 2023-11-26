/**
   ESP32 for joint control of bionic hand
*/



#include <esp_now.h>
#include <WiFi.h>
#include <Servo_ESP32.h>

#define CHANNEL 1
#define LED 33


int angleNum;
int angleVal;

// twelve servo objects can be created on most boards
Servo_ESP32 ThumbAb;  //Ab for Finger Abduction
Servo_ESP32 IndexAb;
Servo_ESP32 MiddleAb;
Servo_ESP32 RingAb;
Servo_ESP32 PinkyAb;
Servo_ESP32 ThumbFlex;  //Flex for Finger Flexion
Servo_ESP32 IndexFlex;
Servo_ESP32 MiddleFlex;
Servo_ESP32 RingFlex;
Servo_ESP32 PinkyFlex;


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
  ThumbFlex.attach(33);   // Thumb Flexion
  IndexFlex.attach(13);   //Index Flexion
  MiddleFlex.attach(26);  //Middle Flexion
  RingFlex.attach(27);    //Ring Flexion
  PinkyFlex.attach(14);   //Pinky Flexion
  ThumbAb.attach(16);     //Thumb Abduction
  IndexAb.attach(17);     //Index Abduction
  MiddleAb.attach(5);     //Middle Abduction
  RingAb.attach(18);      //Ring Abduction
  PinkyAb.attach(19);     //Pinky Abduction

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
  // Register for recv CB to get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  //Set up LED for debugging
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
  // Serial.print(myData.a);
  // Serial.print(", ");
  // Serial.println(myData.b);
  // servoSet(myData.a, myData.b);
  angleNum = myData.a;  //Specifies joint
  angleVal = myData.b;  //Defines servo movement
}

void loop() {
  if (angleNum == 1) {
    int angleVal1 = map(angleVal, 0, 90, 10, 110);
    ThumbFlex.write(angleVal1);
    int angleVal6 = map(angleVal, 0, 90, 20, 130);
    ThumbAb.write(angleVal6);
  }
  if (angleNum == 2) {
    int angleVal2 = map(angleVal, 0, 150, 2, 180);
    IndexFlex.write(angleVal2);
    Serial.println(angleVal2);
  }
  if (angleNum == 3) {
    int angleVal3 = map(angleVal, 0, 130, 170, 2);
    MiddleFlex.write(angleVal3);
  }
  if (angleNum == 4) {
    int angleVal4 = map(angleVal, 0, 140, 170, 2);
    RingFlex.write(angleVal4);
  }
  if (angleNum == 5) {
    int angleVal5 = map(angleVal, 0, 130, 180, 0);
    PinkyFlex.write(angleVal5);
  }
  // Only needed if Thumb flexion is independent in CV code
  // if (angleNum == 6) {
  //   int angleVal6 = map(angleVal, 0, 90, 10, 120);
  //   ThumbAb.write(angleVal6);
  // }
  if (angleNum == 7) {
    int angleVal7 = map(angleVal, 0, 20, 65, 20);
    IndexAb.write(angleVal7);
  }
  if (angleNum == 8) {
    int angleVal8 = map(angleVal, 0, 20, 115, 145);
    MiddleAb.write(angleVal8);
  }
  if (angleNum == 9) {
    int angleVal9 = map(angleVal, 0, 20, 15, 40);
    RingAb.write(angleVal9);
  }
  if (angleNum == 10) {
    int angleVal10 = map(angleVal, 0, 20, 80, 115);
    PinkyAb.write(angleVal10);
  }
  delay(2);
}
