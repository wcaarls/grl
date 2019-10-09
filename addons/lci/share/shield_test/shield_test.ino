#include <EncoderShield.h>
#include <MotoShield.h>

#define MAX_PWM 255

Motor motor(0);
Encoder angle_encoder(0, 2500), pos_encoder(1, 100);
Device end_stop_left(0), end_stop_right(1);

void setup() {
  Serial.begin(115200);

  // 31kHz PWM on pins 2, 3, 5
  TCCR3B &= ~7;
  TCCR3B |= (1 << CS30);

  // 31kHz PWM on pins 6, 7, 8
  TCCR4B &= ~7;
  TCCR4B |= (1 << CS40);

  motor.begin();
  angle_encoder.begin();
  pos_encoder.begin();
  end_stop_left.begin();
  end_stop_right.begin();

  //angle_encoder.home();
  //pos_encoder.home();
}

void loop() {
  static int pwm=0, change=1;

  pwm += change;
  if (abs(pwm) == MAX_PWM)
    change = -change;

  if (pwm > 0)
    motor.go(CCW, pwm);
  else
    motor.go(CW, -pwm);
  
  float angle = angle_encoder.angle();
  float pos = pos_encoder.angle()*0.02;

  Serial.print("p: ");
  Serial.print(pwm);
  Serial.print(", c: ");
  Serial.print(motor.current());
  Serial.print(", a: ");
  Serial.print(angle);
  Serial.print(", p: ");
  Serial.print(pos);
  Serial.print(", l: ");
  Serial.print(end_stop_left.read());
  Serial.print(", r: ");
  Serial.println(end_stop_right.read());

  delay(10);
}
