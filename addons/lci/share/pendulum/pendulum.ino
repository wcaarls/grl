#include <EncoderShield.h>
#include <MotoShield.h>

Motor motor(0);
Encoder encoder(1, 1000);

void setup() {
  Serial.begin(115200);

  // 31kHz PWM on pins 2, 3, 5
  TCCR3B &= ~7;
  TCCR3B |= (1 << CS30);

  motor.begin();
  encoder.begin();
}

void loop() {
  static long int prev_angle = encoder.count();

  unsigned long int last = millis();
  long int angle = encoder.count();
  long int vel   = (angle-prev_angle) * 20;

  // Lowest possible speed is 200s / revolution
  // 4000 ppr, 50ms per step
  Serial.write(0xff);
  Serial.write((unsigned char*)&angle, 4);
  Serial.write((unsigned char*)&vel, 4);

  while (millis() - last < 50)
  {
    if (Serial.available())
    {
      signed char pwm = Serial.read();
      if (pwm > 0)
        motor.go(CCW, pwm);
      else
        motor.go(CW, -pwm);
    }
  }

  prev_angle = angle;
}
