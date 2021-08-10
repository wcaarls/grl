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

  // Make sure we passed index once
  encoder.home();
}

void loop() {
  static long int prev_angle = encoder.count();
  static unsigned int missed = 0;

  unsigned long int last = millis();
  long int angle = encoder.count();
  long int vel   = angle-prev_angle;
  bool received = false;

  // Rotating through index
  if (vel > 2000) vel -= 4000;
  if (vel < -2000) vel += 4000;
  vel *= 20;

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
        motor.go(CW,   pwm);
      else
        motor.go(CCW, -pwm);
      received = true;
    }
  }

  if (!received && missed++ > 10)
      motor.stop();
  
  prev_angle = angle;
}
