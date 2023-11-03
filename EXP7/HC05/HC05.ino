#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2, 3);

void setup() {
  Serial.begin(9600);   
  bluetooth.begin(9600);
}

void loop() { 
  if (bluetooth.available()) {
    char data = bluetooth.read();
    Serial.print(data);
  }
  if (Serial.available()) {
    char data = Serial.read();
    bluetooth.print(data);
  }
}
