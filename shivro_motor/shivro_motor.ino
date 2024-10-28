int motorPin1 = 9; 
int motorPin2 = 10;
volatile int encoderValue = 0;
int encoderPinA = 2; 
int encoderPinB = 3;
int targetAngle = 0;
int currentAngle = 0;
int initialAngle = 0;
int motorSpeed = 150; // Adjust speed as needed
int PPR = 500; // Adjust ppr as needed

void setup() {
  Serial.begin(115200);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, FALLING);
  Serial.println("Enter angle between 0 to 360 degrees: ");
}

void loop() {
  if (Serial.available() > 0) {
    targetAngle = Serial.parseInt();
    if (targetAngle >= 0 && targetAngle <= 360) {
      Serial.print("Input Angle:");Serial.println(targetAngle);
      moveMotorToAngle(targetAngle);
       delay(1000);
       moveMotorToAngle(initialAngle); 
    } else {
      Serial.println("Please enter a valid angle (0-360)");
    }
  }

  currentAngle = calculateAngle();
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
  delay(100);
}

void moveMotorToAngle(int targetAngle) {
  while (currentAngle != targetAngle) {
    currentAngle = calculateAngle();
    Serial.print("ENC Reading:");Serial.print(encoderValue); Serial.print(" Current Angle:");Serial.println(currentAngle);
    if (currentAngle < targetAngle) {
      analogWrite(motorPin1, motorSpeed);
      analogWrite(motorPin2, 0);
    } else if (currentAngle > targetAngle) {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, motorSpeed);
    }
    if (currentAngle == targetAngle) {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, 0);
      Serial.println("Target angle reached!");
      break;
    }
  }
}

int calculateAngle() {
  return (encoderValue * 360) / PPR;
}

void updateEncoder() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderValue++;
  } else {
    encoderValue--;
  }
}
