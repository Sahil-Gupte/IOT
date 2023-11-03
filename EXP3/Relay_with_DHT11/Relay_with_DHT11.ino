#include "DHT.h"
#define DHTPIN 2        
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

int relayPin = 4;

void setup() {
  pinMode(relayPin, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("DHT11 test!"));
  dht.begin();
}

void loop() {
    delay(2000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.println(F("%"));
  Serial.print(F("Temperature: "));
  Serial.print(t);
  Serial.println(F("°C"));

  if (t >= 30) {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
  }
}
