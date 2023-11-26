#include <stdlib.h>  // required for random function
#include <SoftwareSerial.h>
#define sensor1 A6
#define sensor2 A5
#define sensor3 A4
#define sensor4 A3
#define sensor5 A2

#define MaxClosedAngle 180
#define MaxOpenAngle 0

SoftwareSerial Serial1(7, 8);  // RX, TX Pins for ESP8266 connection

//Initialize State Machine
enum State {
  READ_SENSOR1,
  READ_SENSOR2,
  READ_SENSOR3,
  READ_SENSOR4,
  READ_SENSOR5
};

const int closedbuttonPin = 2;   // Pin connection for calibrating closed fist
const int openbuttonPin = 3;     // Pin connection for calibrating open fist
volatile int buttonState = LOW;  // initialize the button state as low
volatile int lastButtonState = LOW;

int ClosedReading1 = 1024;
int OpenReading1 = 0;
int ClosedReading2 = 1024;
int OpenReading2 = 0;
int ClosedReading3 = 1024;
int OpenReading3 = 0;
int ClosedReading4 = 1024;
int OpenReading4 = 0;
int ClosedReading5 = 1024;
int OpenReading5 = 0;

State state = READ_SENSOR1;
int a;
int b;

void setup() {
  // Start serial communication
  pinMode(closedbuttonPin, INPUT);  // set the first button pin as input with internal pull-up resistor
  pinMode(openbuttonPin, INPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  attachInterrupt(digitalPinToInterrupt(closedbuttonPin), ClosedCalibration, CHANGE);
  attachInterrupt(digitalPinToInterrupt(openbuttonPin), OpenCalibration, CHANGE);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  // Set random seed based on the value of the analog pin 0
  randomSeed(analogRead(0));
}

void loop() {
  // Generate a random angle between 0-180 degrees
  switch (state) {

    case READ_SENSOR1:
      // Read value of sensor 1
      a = 1;
      b = analogRead(sensor1);
      b = map(b, OpenReading1, ClosedReading1, MaxOpenAngle, MaxClosedAngle);
      b = map(sensor1, 0, 180, 0, 90);
      Serial.println(b);
      state = READ_SENSOR2;
      break;

    case READ_SENSOR2:
      // Read value of sensor 2
      a = 2;
      b = analogRead(sensor2);
      b = map(b, OpenReading2, ClosedReading2, MaxOpenAngle, MaxClosedAngle);
      state = READ_SENSOR3;
      break;

    case READ_SENSOR3:
      // Read value of sensor 3
      a = 3;
      b = analogRead(sensor3);
      b = map(b, OpenReading3, ClosedReading3, MaxOpenAngle, MaxClosedAngle);
      state = READ_SENSOR4;
      break;

    case READ_SENSOR4:
      // Read value of sensor 4
      a = 4;
      b = analogRead(sensor4);
      b = map(b, OpenReading4, ClosedReading4, MaxOpenAngle, MaxClosedAngle);
      state = READ_SENSOR5;
      break;

    case READ_SENSOR5:
      // Read value of sensor 5
      a = 5;
      b = analogRead(sensor5);
      b = map(b, OpenReading5, ClosedReading5, MaxOpenAngle, MaxClosedAngle);
      state = READ_SENSOR1;
      break;
  }
  //After every reading, send information
  if (b < 0) { b = 0; }
  if (b > 180) { b = 180; }
  sendData(a, b);
  delay(10);
}

void sendData(int a, int b) {
  Serial1.print(a);
  Serial1.print(", ");
  Serial1.println(b);
}

void ClosedCalibration() {
  // read the current state of the button
  if (digitalRead(closedbuttonPin) == HIGH) {

    ClosedReading1 = 0;
    ClosedReading2 = 0;
    ClosedReading3 = 0;
    ClosedReading4 = 0;
    ClosedReading5 = 0;

    for (int i = 0; i <= 4; i++) {
      ClosedReading1 += analogRead(sensor1) / 5;
      ClosedReading2 += analogRead(sensor2) / 5;
      ClosedReading3 += analogRead(sensor3) / 5;
      ClosedReading4 += analogRead(sensor4) / 5;
      ClosedReading5 += analogRead(sensor5) / 5;
      delay(20);
    }

    Serial.println("Calibrated Closed Hand!");
  }
}

void OpenCalibration() {

  if (digitalRead(openbuttonPin) == HIGH) {
    // read the current state of the button
    OpenReading1 = 0;
    OpenReading2 = 0;
    OpenReading3 = 0;
    OpenReading4 = 0;
    OpenReading5 = 0;

    for (int i = 0; i <= 4; i++) {
      OpenReading1 += analogRead(sensor1) / 5;
      OpenReading2 += analogRead(sensor2) / 5;
      OpenReading3 += analogRead(sensor3) / 5;
      OpenReading4 += analogRead(sensor4) / 5;
      OpenReading5 += analogRead(sensor5) / 5;
      delay(20);
    }

    Serial.println("Calibrated Open Hand!");
  }
}
