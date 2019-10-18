#include <EncoderShield.h>
#include <MotoShield.h>

#define MAX_PWM 255

Motor motor(0);
Encoder angle_encoder(0, 1000, -689+714+2000), pos_encoder(1, 100);
Device end_stop_left(0), end_stop_right(1);

long int offset=0;

void setup() {
  // 31kHz PWM on pins 2, 3, 5
  TCCR3B &= ~7;
  TCCR3B |= (1 << CS30);

  // 31kHz PWM on pins 6, 7, 8
  TCCR4B &= ~7;
  TCCR4B |= (1 << CS40);
  
  Serial.begin(115200);  

  motor.begin();
  angle_encoder.begin();
  pos_encoder.begin();
  end_stop_left.begin();
  end_stop_right.begin();

  Serial.println("Homing");

  angle_encoder.home();
  offset = pos_encoder.count();

  Serial.println("Starting");
}

#define P_ANGLE 20
#define D_ANGLE 0.1
#define P_POS   -5
#define D_POS   -0.3
#define ALPHA 0.95

void loop() {
  static float prevangerr=0, prevposerr=0, filtangder=0, filtposder=0;

  float angerr = angle_encoder.angle();
  float poserr = (pos_encoder.count()-offset)/4000.;

  float angder = (angerr-prevangerr)/0.001;
  float posder = (poserr-prevposerr)/0.001;

  filtangder = ALPHA*filtangder+(1-ALPHA)*angder;
  filtposder = ALPHA*filtposder+(1-ALPHA)*posder;
  
  float v = P_ANGLE*angerr + D_ANGLE*filtangder + P_POS*poserr + D_POS*filtposder;

  if (v > 1) v = 1;
  if (v < -1) v = -1;
  
  uint8_t dir = (v<0)?CW:(v>0)?CCW:BRAKEGND;
  uint8_t pwm = map((long int)(1024*abs(v)), 0, 1024, 25, 250);

  motor.go(dir, pwm);

  prevangerr = angerr;
  prevposerr = poserr;

  delay(1);
}
