/*  MonsterMoto Shield Example Sketch
  date: 5/24/11
  code by: Jim Lindblom
  hardware by: Nate Bernstein
  SparkFun Electronics

 This is really simple example code to get you some basic
 functionality with the MonsterMoto Shield. The MonsterMote uses
 two VNH2SP30 high-current full-bridge motor drivers.
 
 Use the motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) 
 function to get motors going in either CW, CCW, BRAKEVCC, or 
 BRAKEGND. Use motorOff(int motor) to turn a specific motor off.
 
 The motor variable in each function should be either a 0 or a 1.
 pwm in the motorGo function should be a value between 0 and 255.
 
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
 */
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
static int inApin[2] = {7, 4};  // INA: Clockwise input
static int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
static int pwmpin[2] = {5, 6}; // PWM input
static int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
static int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

class Motor
{
  protected:
    uint8_t id_;

  public:
    Motor(uint8_t id) : id_(id)
    {
      if (id_ > 1)
        id_ = 1;
    }
    
    void begin()
    {
      // Initialize digital pins as outputs
      pinMode(inApin[id_], OUTPUT);
      pinMode(inBpin[id_], OUTPUT);
      pinMode(pwmpin[id_], OUTPUT);
      
      // Initialize braked
      stop();
    }
    
    /* go() will set a motor going in a specific direction
     the motor will continue going in that direction, at that speed
     until told to do otherwise.
     
     dir: Should be between 0 and 3, with the following result
     0: Brake to VCC
     1: Clockwise
     2: CounterClockwise
     3: Brake to GND
     
     pwm: should be a value between ? and 1023, higher the number, the faster
     it'll go
     */
    void go(uint8_t dir, uint8_t pwm)
    {
      if (dir <=4)
      {
        // Set inA[motor]
        if (dir <=1)
          digitalWrite(inApin[id_], HIGH);
        else
          digitalWrite(inApin[id_], LOW);

        // Set inB[motor]
        if ((dir==0)||(dir==2))
          digitalWrite(inBpin[id_], HIGH);
        else
          digitalWrite(inBpin[id_], LOW);

        analogWrite(pwmpin[id_], pwm);
      }
    }
    
    void stop()
    {
      go(BRAKEGND, 0);
    }
    
    unsigned int current()
    {
      return analogRead(cspin[id_]);
    }
};
