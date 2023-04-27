#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define interruptPin 0

// Define the software serial pins
SoftwareSerial espSerial(5, 4);  // RX, TX

uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

int state = 0;

typedef struct struct_message {
  int a;
  uint8_t b;
} struct_message;
struct_message myData;

void ICACHE_RAM_ATTR IntCallback() {
  Serial.println("Change State");
  Serial.flush();
  if (state == 0) {
    state = 1;
    digitalWrite(2, HIGH);
    delay(400);
  } else if (state == 1) {
    state = 0;
    digitalWrite(2, LOW);
    delay(400);
  }
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Start serial communication

  Serial.begin(115200);
  espSerial.begin(115200);

  Serial.println("Hello");
  Serial.print("1");
  Serial.flush();
  attachInterrupt(interruptPin, IntCallback, FALLING);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  WiFi.mode(WIFI_STA);
  esp_now_init();  //Needed
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);  //check to see if I can input MAC Address or not
}

void loop() {
  if (state == 0) {
    while (espSerial.available() > 0) {
      String message = espSerial.readStringUntil('\n');
      // parse the angle number and value
      int angleNum = message.substring(0, message.indexOf(' ')).toInt();
      int angleVal = message.substring(message.indexOf(' ') + 1).toInt();
      // print the angle number and value
      if (angleNum == 1) {
        int angleVal1 = map(angleVal, 0, 180, 0, 90); //values for recalibrating (val, min input, max input, min output, max output)
        myData.b = angleVal1;
      }
      if (angleNum == 2) {
        int angleVal2 = map(angleVal, 0, 180, 0, 130);
        myData.b = angleVal2;
      }
      if (angleNum == 3) {
        int angleVal3 = map(angleVal3, 0, 180, 0, 130);
        myData.b = angleVal3;
      }
      // if (angleNum == 4) {
      //   servo4.write(angleVal);
      // myData.b = angleVal4;
      // }
      if (angleNum == 5) {
        int angleVal5 = map(angleVal, 0, 180, 0, 130);
        myData.b = angleVal5;
      }

      myData.a = angleNum;
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      // Serial.print(angleNum);
      // Serial.print(", ");
      // Serial.println(angleVal);
    }
  }

  if (state == 1) {
    while (Serial.available() > 0) {
      String message = Serial.readStringUntil('\n');
      // parse the angle number and value
      int angleNum = message.substring(0, message.indexOf(' ')).toInt();
      int angleVal = message.substring(message.indexOf(' ') + 1).toInt();
      // print the angle number and value
      myData.a = angleNum;
      myData.b = angleVal;
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    }
  }
}
