void setup() {
  pinMode(12, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  int val= digitalRead(12);
  Serial.println(val);
  delay(500);
}
