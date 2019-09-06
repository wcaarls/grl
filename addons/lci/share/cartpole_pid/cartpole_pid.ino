#define PIN_DIR      6
#define PIN_PWM      7

#define PIN_ANGLE_A 19
#define PIN_ANGLE_B 20
#define PIN_ANGLE_Z 21

#define PIN_POS_A    2
#define PIN_POS_B    3

#define ANGLE_ZERO -689

#define NANGLE_ONLY_A

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
#ifndef ANGLE_ONLY_A
  attachInterrupt(digitalPinToInterrupt(20), angleEncoderB, CHANGE);
#endif
  attachInterrupt(digitalPinToInterrupt(21), angleEncoderZ, RISING);
  attachInterrupt(digitalPinToInterrupt(2), posEncoderA, CHANGE);

  digitalWrite(PIN_DIR, 0);
  analogWrite(PIN_PWM, 0);

  while (abs(angle) > 100) delay(10);
  pos = 0;

  Serial.begin(115200);  
}

#define P_ANGLE 30
#define D_ANGLE 1
#define P_POS   5
#define D_POS   3
#define ALPHA 0.95

void loop() {
  noInterrupts();
  long int _angle = angle, _pos = pos;
  interrupts();

  static float prevangerr=0, prevposerr=0, filtangder=0, filtposder=0;
  float angerr = _angle*2*M_PI/4000, poserr = _pos/4000.;
  float angder = (angerr-prevangerr)/0.001;
  float posder = (poserr-prevposerr)/0.001;

  filtangder = ALPHA*filtangder+(1-ALPHA)*angder;
  filtposder = ALPHA*filtposder+(1-ALPHA)*posder;
  
  float v = P_ANGLE*angerr + D_ANGLE*filtangder + P_POS*poserr + D_POS*filtposder;

  if (v > 1) v = 1;
  if (v < -1) v = -1;

  long int intv = map((long int)(1024*abs(v)), 0, 1024, 0, 250);
  digitalWrite(PIN_DIR, v<0);

  if (abs(angle) > 200)
  {
    analogWrite(PIN_PWM, 0);
    pos = 0;
  }
  else
    analogWrite(PIN_PWM, intv);

  //Serial.print(angle);
  //Serial.print(" ");
  //Serial.print(angerr);
  //Serial.print(" ");
  //Serial.print(angder);
  //Serial.print(" ");
  //Serial.println(intv);


//  Serial.print("a: ");
//  Serial.print(angerr);
//  Serial.print(", p: ");
//  Serial.println(poserr);

  prevangerr = angerr;
  prevposerr = poserr;

  delay(1);
}

#ifdef ANGLE_ONLY_A
void angleEncoderA()
{
  bool PastA = digitalRead(PIN_ANGLE_A) == HIGH;
  bool PastB = digitalRead(PIN_ANGLE_B) == HIGH;
  angle += (PastA != PastB) ? +1 : -1;
}
#else
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

#endif


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
