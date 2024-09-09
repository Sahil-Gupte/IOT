//example input 1000(rpm), 1/3(feeding-->on/off) , 1/2/3(lead screw-->reverse,forward,kill)

#define M_DIR1 6  //top-wheel1
#define M_IP1 PC_7 //top-wheel1

#define M_DIR2 6 //top-wheel2
#define M_IP2 PC_6 //top-wheel2

#define DEG_TO_RAD(x) (x * 0.0175)
#define RAD_TO_DEG(x) (x / 0.0175)

#define test(x) sin(x)

#define LIM PD_6
#define LIM2 PD_7

//#define LASER
//#define LED BLUE_LED

#define SERVO 12 //servo


#define M_DIR3 6  //feeding wheel dir 1
#define M_IP3 7 //feeding wheel pwm 1

#define M_DIR4 PF_0  // lead screw dir 2
#define M_IP4 PC_4 //lead screw pwm 2

#define IR_IP A5

////solenoid
#define SOL_IP1 21
#define SOL_IP2 20

//encoder-interrupt
#define CLK PB_2 //wheel1-rpm 
#define CLK2 PB_3 //wheel2
//rpm
#define LS_DIST PB_0 //encoderA
#define LS_DIR PB_5 //encoderB

bool dir_flag, ls_pos, ip_flag = false, ip_flag2 = false;
volatile long wheel_angle, counter, old_counter;
unsigned long l_timer = 0, l_timer2 = 0;
int InByte, InByte2, InByte3, InitReading, InitReading2, idealRPM = 0;
unsigned long timer = 0, timer1 = 0, timer2 = 0, timer3 = 0, up_timer = 0, down_timer = 0, act_timer = 0, deact_timer = 0, angletimer = 0;

float error, error2, kp, pid, pid2, last_error, last_error2, kd, D, D2;
int M1_nPWM, M2_nPWM;
int l_count;

int MIN_RPM, MAX_RPM = 1000, M1_PWM, M2_PWM;
volatile long RPM_counter1 = 0, RPM_counter2 = 0, dist_counter = 0;
long RPM1 = 0 , RPM2 = 0, currentRPM1, currentRPM2, newRPM1, newRPM2;
bool flag  = false, flag2 = false;
unsigned long uu = 0, ww = 0, zz = 0, feed_timer = 0, feed_timerB = 0;
bool test_flag = false;
bool test_flagB = false;

//Angle
float A, AD = 216, C = DEG_TO_RAD(85.33), DC;

//FLAP 1
#define FLAP_ISR PF_2
#define FLAP_DIR PF_3
#define FLAP_MDIR PD_0
#define FLAP_MPWM PD_1

//FLAP 2
#define FLAPB_MDIR PD_2
#define FLAPB_MPWM PD_1

bool ligma = true , flapdir_flag = true;
boolean flap = false;

volatile unsigned long teamer, taimur, taimurR;

volatile long isr_count, isr_countb = 0, flap_count = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);
  Serial.println("INIT");
  analogWrite(M_IP2, 0);
  pinMode(M_DIR1 , OUTPUT);
  pinMode(M_DIR2 , OUTPUT);
  pinMode(M_DIR3 , OUTPUT);
  pinMode(M_DIR4 , OUTPUT);
  pinMode(SERVO , OUTPUT);

  //  pinMode(LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  //pinMode(LASER , OUTPUT);

  pinMode(M_IP1 , OUTPUT);
  pinMode(M_IP2 , OUTPUT);
  pinMode(M_IP3 , OUTPUT);
  pinMode(M_IP4 , OUTPUT);
  pinMode(LIM, INPUT_PULLUP);
  pinMode(LIM2, INPUT_PULLUP);


  pinMode(SOL_IP1 , OUTPUT);
  pinMode(SOL_IP2 , OUTPUT);

  pinMode(CLK,  INPUT_PULLUP);
  pinMode(CLK2,  INPUT_PULLUP);
  pinMode(LS_DIST, INPUT_PULLUP);
  pinMode(LS_DIR, INPUT_PULLUP);

  //Flap
  pinMode(FLAP_MDIR, OUTPUT);
  pinMode(FLAP_MPWM, OUTPUT);
  pinMode(FLAPB_MDIR, OUTPUT);
  pinMode(FLAPB_MPWM, OUTPUT);
  pinMode(FLAP_ISR, INPUT_PULLUP);
  pinMode(FLAP_DIR, INPUT_PULLUP);


  //pinMode(LASER, OUTPUT); //IR Laser
  pinMode(IR_IP, INPUT_PULLUP);


  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoderA, RISING); //wheel1
  attachInterrupt(digitalPinToInterrupt(CLK2), updateEncoderB, RISING); //wheel2

  timer = millis();
  angletimer = millis();
  timer1 = millis();
  taimur = micros();
  taimurR = micros();

  analogWrite(M_IP1, 0);
  analogWrite(M_IP3, 0);
  analogWrite(M_IP4, 0);

  //  while (digitalRead(LIM) == 1) {
  //    Serial.println("Moving back to initial position");
  //    digitalWrite(M_DIR4, HIGH);
  //    analogWrite(M_IP4, 100);
  //  }

  //  if (digitalRead(LIM) == 0)
  //  { Serial.println("At INIT POS");
  //    digitalWrite(M_DIR4, LOW);
  //    digitalWrite(M_IP4, LOW);
  //    counter = 0;Serial.println(digitalRead(LIM));
  //  }
  attachInterrupt(digitalPinToInterrupt(LS_DIST), lead, FALLING);
  //  attachInterrupt(digitalPinToInterrupt(LIM), leadscrew_limit1, FALLING);
  //  attachInterrupt(digitalPinToInterrupt(LIM2), leadscrew_limit2, FALLING);
}
void loop() {
  rpm();
  gateway();
  angle();


  digitalWrite(GREEN_LED, HIGH);
  if (Serial.available() > 3) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    InByte = Serial.parseInt();
    InByte2 = Serial.parseInt();
    InByte3 = Serial.parseInt();
  }
  if (InByte >= 1 && InByte < 7) {
    set_range();
    InByte = 0;
  }
  if (InByte == 9) {
    idealRPM = 0;
  }

  if (InByte2 >= 5 && InByte <= 45) {
    set_angle();
    InByte2 = 0;
  }

  if (millis() - timer1 > 10) {
    set_pid();
    timer1 = millis();
  }
}

void angle() {
  float dist_trav = (8 * counter) / 1024;

  DC = dist_trav;
  float temp = (((DC * sin(C)) / AD));
  A = RAD_TO_DEG(asin(temp));

  Serial.print("Counter: ");
  Serial.print(counter);
  Serial.print("\tDistance: ");
  Serial.print(dist_trav);
  Serial.print("\tAngle: ");
  Serial.println(A);
}

void lead() {
  if (digitalRead(LS_DIR) == 0) {
    dir_flag = true;
    counter--;
  } else {
    dir_flag = false;
    counter++;
  }
}

void set_angle() {
  int targetAngle = InByte2;
}

void gateway() {
  if (digitalRead(FLAP_DIR) == 0) {
    flapdir_flag = true;
    flap_count--;
  } else {
    flapdir_flag = false;
    flap_count++;
  }
}

void set_range() {
  int range = InByte;
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  double value, r_angle;
  Serial.println("Calculating RPM required...");
  r_angle = DEG_TO_RAD(11);

  float denom = ( (2 * range * 9.81) * (0.01 * cos(r_angle)) * (0.01 * sin(r_angle)) + ((8.829) * (0.01 * cos(r_angle)) * (0.01 * cos(r_angle)) ) ) ;
  value = (range * 9.81) * (range * 9.81) / denom;

  idealRPM = sqrt(value) * 1.5;

  Serial.print("\tRPM calculated: ");
  Serial.println(idealRPM);
  digitalWrite(BLUE_LED, LOW);
}

void leadscrew_limit1() {
  isr_count++;
  if (isr_count > 12) {
    analogWrite(M_IP4, 0);
    isr_count = 0;
    isr_countb = 0;
  }
}

void leadscrew_limit2() {
  isr_countb++;
  if (isr_countb > 12) {
    analogWrite(M_IP4, 0);
    isr_count = 0;
    isr_countb = 0;
  }
}

void servo(int servo, int angle) {
  int del = (11 * angle) + 500;
  digitalWrite(SERVO, HIGH);
  delayMicroseconds(del);
  digitalWrite(SERVO, LOW);
}

void updateEncoderA() {
  RPM_counter1 ++;
}

void updateEncoderB() {
  RPM_counter2 ++;
}

void rpm() {
  if (millis() - timer > 10) {
    noInterrupts();
    RPM1 = RPM_counter1 * 5.8 ; //PPRC = 10 seconds / PPR;
    RPM2 = RPM_counter2 * 5.8;
    timer = millis();
    RPM_counter1 = 0;
    RPM_counter2 = 0;
    interrupts();
  }
  currentRPM1 = RPM1;
  currentRPM2 = RPM2;
}

void pwm(int pwm_pin, uint8_t Speed) {
  analogWrite(pwm_pin, Speed);
  if (millis() - timer3 > 10)
  {
    //        Serial.print("PWM:");
    //        Serial.print(Speed);
    //        Serial.print("\tRPM-1:");
    //        Serial.print(RPM1);
    //        Serial.print("\tRPM-2:");
    //        Serial.println(RPM2);
    Serial.flush();
    timer3 = millis();
  }
}

void up() {
  digitalWrite(M_DIR3, HIGH);
  analogWrite(M_IP3 , 200);
  Serial.println("Moving up");
  Serial.println(digitalRead(IR_IP));
  flag  = true;
}

void down() {
  digitalWrite(M_DIR3, LOW);
  analogWrite(M_IP3 , 200);
  Serial.println("Moving down");
  if (!digitalRead(IR_IP)) {
    //analogWrite(M_IP3 , 0);
    Serial.println("Stopping");
  }
}

void kill() {
  digitalWrite(M_IP3, LOW);
}

void set_pid() {

  kp = 0.3;
  kd = 0.75;
  newRPM1 = idealRPM;
  newRPM2 = idealRPM;
  error = newRPM1 - currentRPM1;
  error2 = newRPM2 - currentRPM2;

  D = (error - last_error) / 10;
  D2 = (error2 - last_error2) / 10;

  pid = (error * kp) + (D * kd);
  pid2 = (error2 * kp) + (D2 * kd);

  if (pid > 1)
    pid = 1;
  if (pid < -1)
    pid = -1;

  if (pid2 > 1)
    pid2 = 1;
  if (pid2 < -1)
    pid2 = -1;

  M1_nPWM = M1_nPWM + pid;
  if (M1_nPWM > 255)
    M1_nPWM = 255;
  if (M1_nPWM < 0)
    M1_nPWM = 0;
  pwm(M_IP1, M1_nPWM);

  M2_nPWM = M2_nPWM + pid2;
  if (M2_nPWM > 255) {
    M2_nPWM = 255;
  } else if (M2_nPWM < 0) {
    M2_nPWM = 0;
  }

  pwm(M_IP2, M2_nPWM);

  last_error = error;
  last_error2 = error2;
}
