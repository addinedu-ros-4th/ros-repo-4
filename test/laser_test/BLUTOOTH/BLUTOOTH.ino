#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3); // RX to 2, TX to 3
int laser = 13;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  pinMode(laser, OUTPUT);
}

void loop() {
  if (BTSerial.available()) {
    char receivedData[4] = {0}; // 크기 4로 변경 (3문자 + NULL)
    if (BTSerial.readBytes(receivedData, 3) == 3) { // 정확히 3바이트를 읽기
      if (strcmp(receivedData, "cow") == 0) {
        digitalWrite(laser, HIGH);
      }
      else if(strcmp(receivedData,"end")== 0){
        digitalWrite(laser,LOW);
      }
      Serial.print("Received: ");
      Serial.println(receivedData);
    }
  }

  if (Serial.available()) {
    char sendChar = Serial.read();
    BTSerial.print(sendChar);
  }
}
