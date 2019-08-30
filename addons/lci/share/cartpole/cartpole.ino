#define PIN_DIR      6
#define PIN_PWM      7

#define PIN_ANGLE_A 19
#define PIN_ANGLE_B 20
#define PIN_ANGLE_Z 21

#define PIN_POS_A    2
#define PIN_POS_B    3

#define ANGLE_ZERO -689

long int angle=-10000, pos=0;

void setup() {
  TCCR4B &= ~7;
  TCCR4B |= (1 << CS40);
  
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  pinMode(PIN_ANGLE_A, INPUT);
  pinMode(PIN_ANGLE_B, INPUT);
  pinMode(PIN_ANGLE_Z, INPUT);

  pinMode(PIN_POS_A, INPUT);
  pinMode(PIN_POS_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(19), angleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), angleEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), angleEncoderZ, RISING);
  attachInterrupt(digitalPinToInterrupt(2), posEncoderA, CHANGE);

  digitalWrite(PIN_DIR, 0);
  analogWrite(PIN_PWM, 0);

  while (abs(angle) > 100) delay(10);
  pos = 0;

  Serial.begin(115200);  
}

typedef struct __attribute__((packed)) {
  unsigned char header;
  float poserr, angerr;
} msg_t;

void loop() {
  noInterrupts();
  long int _pos = pos, _angle = angle;
  interrupts();

  msg_t msg;
  msg.header = 0xFF;
  msg.poserr = _pos/4000.;
  msg.angerr = _angle*2*M_PI/4000;

  // Send state
  Serial.write((unsigned char*)&msg, sizeof(msg_t));

  long int t = millis();
  int n = 0;
  unsigned char c=0;
  static long int idle_time = 0;
  bool received = false;

  // Receive action
  while (Serial.available() && t < millis() + 10)
  {
    unsigned char c=0;
    while (c != 0xFF && t < millis() + 10)
      if (Serial.available())
        c = Serial.read();

    if (c == 0xFF)
    {
      unsigned char buf[4], n = 0;
      while (n < 4 && t < millis() + 10)
        if (Serial.available())
          buf[n++] = Serial.read();

      if (n == 4)
      {
        float action = *(float*)buf;
        digitalWrite(PIN_DIR, action>0);
        analogWrite(PIN_PWM, map(1024*fabs(action), 0, 1024, 0, 250));
        received = true;
      }
    }
  }

  if (!received)
  {
    if (idle_time++ > 10)
      analogWrite(PIN_PWM, 0);
  }
  else
    idle_time = 0;

  static long int timeout = millis();
  delay(timeout-millis());
  timeout = millis() + 10;
}

void angleEncoderA()
{
  bool PastA = digitalRead(PIN_ANGLE_A) == HIGH;
  bool PastB = digitalRead(PIN_ANGLE_B) == HIGH;
  
  angle += (PastA != PastB) ? +1 : -1;
}

void angleEncoderB()
{
  bool PastA = digitalRead(PIN_ANGLE_A) == HIGH;
  bool PastB = digitalRead(PIN_ANGLE_B) == HIGH;
  
  angle += (PastA != PastB) ? -1 : +1;
}

void angleEncoderZ()
{
  angle = ANGLE_ZERO;
}

void posEncoderA()
{
  bool PastA = digitalRead(PIN_POS_A) == HIGH;
  bool PastB = digitalRead(PIN_POS_B) == HIGH;
  pos += (PastA != PastB) ? +1 : -1;
}
