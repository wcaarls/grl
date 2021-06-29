// WARNING:
// Arduino Mega2560 has 10k pull-up on pins 21 and 20. Remove for
// optimal performance.

#define PIN_ENCODER0_A 21 // PD0
#define PIN_ENCODER0_B 20 // PD1
#define PIN_ENCODER0_Z 19 // PD2

#define PIN_ENCODER1_A 18 // PD3
#define PIN_ENCODER1_B 2  // PE4
#define PIN_ENCODER1_Z 3  // PE5

static int encApin[2] = {PIN_ENCODER0_A, PIN_ENCODER1_A};
static int encBpin[2] = {PIN_ENCODER0_B, PIN_ENCODER1_B};
static int encZpin[2] = {PIN_ENCODER0_Z, PIN_ENCODER1_Z};

static int devSpin[8] = {A8, A9, A10, A11, A12, A13, A14, A15};
static int devTpin[4] = {A4, A5, A6, A7};

static volatile long int encoder0_count__ = 0, encoder1_count__ = 0;
static int encoder0_zero__ = 0, encoder1_zero__ = 0;

static volatile long int *count__[2] = {&encoder0_count__, &encoder1_count__};
static int *zero__[2] = {&encoder0_zero__, &encoder1_zero__};

void encoder0A()
{
  bool PastA = (PIND & 1) == 1; // digitalRead(PIN_ENCODER0_A) == HIGH;
  bool PastB = (PIND & 2) == 2; // digitalRead(PIN_ENCODER0_B) == HIGH;

  encoder0_count__ += (PastA != PastB) ? +1 : -1;
}

void encoder0B()
{
  bool PastA = (PIND & 1) == 1; // digitalRead(PIN_ENCODER0_A) == HIGH;
  bool PastB = (PIND & 2) == 2; // digitalRead(PIN_ENCODER0_B) == HIGH;

  encoder0_count__ += (PastA != PastB) ? -1 : +1;
}

void encoder0Z()
{
  encoder0_count__ = encoder0_zero__;
}

void encoder1A()
{
  bool PastA = (PIND & 8) == 8;   // digitalRead(PIN_ENCODER1_A) == HIGH;
  bool PastB = (PINE & 16) == 16; // digitalRead(PIN_ENCODER1_B) == HIGH;
  
  encoder1_count__ += (PastA != PastB) ? +1 : -1;
}

void encoder1B()
{
  bool PastA = (PIND & 8) == 8;   // digitalRead(PIN_ENCODER1_A) == HIGH;
  bool PastB = (PINE & 16) == 16; // digitalRead(PIN_ENCODER1_B) == HIGH;

  encoder1_count__ += (PastA != PastB) ? -1 : +1;
}

void encoder1Z()
{
  encoder1_count__ = encoder1_zero__;
}

class Encoder
{
  protected:
    uint8_t id_;
    unsigned int ppr_;
    volatile long int *count_;

  public:
    Encoder(uint8_t id, unsigned int ppr, int zero=0) : id_(id), ppr_(ppr)
    {
      if (id_ > 1)
        id_ = 1;
        
      *zero__[id_] = zero;
      count_ = count__[id_];
    }
    
    void begin()
    {
      pinMode(encApin[id_], INPUT);
      pinMode(encBpin[id_], INPUT);
      pinMode(encZpin[id_], INPUT);
      
      if (id_ == 0)
      {
        attachInterrupt(digitalPinToInterrupt(encApin[id_]), encoder0A, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encBpin[id_]), encoder0B, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encZpin[id_]), encoder0Z, RISING);
      }
      else
      {
        attachInterrupt(digitalPinToInterrupt(encApin[id_]), encoder1A, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encBpin[id_]), encoder1B, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encZpin[id_]), encoder1Z, RISING);
      }
    }
    
    void home(unsigned int tolerance=100)
    {
      noInterrupts();
      *count_ = -10000;
      interrupts();
      
      while (abs(count()) > tolerance)
        delay(10);
    }
    
    long int count()
    {
      long int _count;
      noInterrupts();
      _count = *count_;
      interrupts();
      return _count;
    }
    
    float angle()
    {
      return 0.5*M_PI*count()/ppr_;
    }
};

class Device
{
  protected:
    uint8_t id_;
    
  public:
    Device(uint8_t id) : id_(id)
    {
      if (id_ > 7)
        id_ = 7;
    }
    
    void begin()
    {
      pinMode(devSpin[id_], INPUT);
      if (id_ > 3)
        pinMode(devTpin[id_-4], OUTPUT);
    }
    
    void write(bool state)
    {
      if (id_ > 3)
        digitalWrite(devTpin[id_-4], state);
    }
    
    bool read()
    {
      return digitalRead(devSpin[id_]);
    }
    
    unsigned int analogRead()
    {
      return ::analogRead(devSpin[id_]);
    }
};

