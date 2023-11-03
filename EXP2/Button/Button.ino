const int bp = 8;
const int lp = 10;
int bs = 0;
void setup() {
  pinMode(lp , OUTPUT);
  pinMode(bp , INPUT);
  Serial.begin(115200);
}

void loop() {
  bs = digitalRead(bp);
  if (bs = HIGH)
  {
    digitalWrite(lp , HIGH);
    Serial.print("Button Pressed");
  }
  else
  {
    digitalWrite(lp , LOW);
    Serial.print("Button Not Pressed");
  }
}
