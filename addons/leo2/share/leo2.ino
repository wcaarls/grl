#define ADDR_GOAL_POSITION_L      30
#define ADDR_PRESENT_POSITION_L   36

#define RETURN_LEVEL_READ          1

#define MODE_IDLE                  0
#define MODE_RIGHTING              1
#define MODE_AWAIT_CONTROL         2
#define MODE_PARKING               3
#define MODE_CONTROL               4

#define PIN_BUTTON_RESET          16
#define PIN_BUTTON_START          17

#define PIN_LED_ERROR             18
#define PIN_LED_CONTROL           19
#define PIN_LED_ARM_MOVING        20

#define ARM_SPEED                  2
#define ALLOWED_POSITION_ERROR     2
#define FALL_ANGLE               512

#define STEP_TIME             100000 // in microseconds

#define NUM_ACTUATORS              1 ///< Controllable motors (4)
#define NUM_MOTORS                 1 ///< All motors (5)
#define NUM_ENCODERS               1 ///< All encoders (8, includes motors)

struct DynamixelInfo
{
  byte id;
  int cw_limit, ccw_limit, torque;
  int pos, speed, temp;
};

DynamixelInfo gDxlInfo[] = 
{{1, 1024, 3072, 0, 0, 0, 0}, ///< Left hip
 {2, 1024, 3072, 0, 0, 0, 0}, ///< Right hip
 {3, 1024, 3072, 0, 0, 0, 0}, ///< Left knee
 {4, 1024, 3072, 0, 0, 0, 0}, ///< Right knee
 {5,    0,    1, 0, 0, 0, 0}, ///< Arm
 {6,    0,    0, 0, 0, 0, 0}, ///< Roll encoder
 {7,    0,    0, 0, 0, 0, 0}, ///< Pitch encoder
 {8,    0,    0, 0, 0, 0, 0}  ///< Yaw encoder
};

enum DynamixelInfos {diHipLeft, diHipRight, diKneeLeft, diKneeRight, diArm, diEncoderRoll, diEncoderPitch, diEncoderYaw};

int gMode;              ///< State machine
volatile byte gStep;    ///< Step indicator
Dynamixel gDxl(3);      ///< Dynamixels on OpenCM 485 bus
Dynamixel gPC(1);       ///< PC connection on OpenCM 9.04 bus
HardwareTimer Timer(1); ///< Step timer

void setup() {
  Serial1.begin(57600);
  gDxl.begin(1); // 57600
  gPC.begin(1); // 57600

  // Initialize button and LED pins  
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(PIN_BUTTON_RESET, INPUT); 
  pinMode(PIN_BUTTON_START, INPUT); 
  pinMode(PIN_LED_ERROR, OUTPUT); 
  pinMode(PIN_LED_CONTROL, OUTPUT); 
  pinMode(PIN_LED_ARM_MOVING, OUTPUT); 

  // Initalize dynamixels
  init_dxl_comm();  

  // Initialize step timer
  Timer.pause();
  Timer.setPeriod(STEP_TIME);
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  Timer.attachInterrupt(TIMER_CH1, stepHandler);
  Timer.refresh();
  Timer.resume();
  
  gMode = MODE_IDLE;
  gStep = 0;
  
  digitalWrite(BOARD_LED_PIN, 0);
}

void init_dxl_comm()
{
  // Wait for all dynamixels to be ready
  for (int ii=0; ii < NUM_ENCODERS; ++ii)
  {
    // Show which dynamixel we're stuck on
    digitalWrite(PIN_LED_ERROR, (ii&1)==0);
    digitalWrite(PIN_LED_CONTROL, (ii&2)==0);
    digitalWrite(PIN_LED_ARM_MOVING, (ii&4)==0);
    
    while (gDxl.ping(gDxlInfo[ii].id) == 0xffff);
  }
  
  // Only reply on READ commands
  gDxl.returnLevel(BROADCAST_ID, RETURN_LEVEL_READ);
  gDxl.setLibStatusReturnLevel(RETURN_LEVEL_READ);

  // Setup angle limits
  for (int ii=0; ii < NUM_MOTORS; ++ii)
  {
    gDxl.cwAngleLimit(gDxlInfo[ii].id, gDxlInfo[ii].cw_limit);
    gDxl.ccwAngleLimit(gDxlInfo[ii].id, gDxlInfo[ii].ccw_limit);
  }
}

void standup()
{ 
  // Go limp, and move arm down
  gDxl.torqueDisable(BROADCAST_ID);
  gDxl.goalPosition(gDxlInfo[diArm].id, gDxlInfo[diArm].cw_limit);
  gDxl.goalSpeed(gDxlInfo[diArm].id, ARM_SPEED);
  gDxl.torqueEnable(gDxlInfo[diArm].id);
}

void readState()
{
  gDxl.setTxPacketParameter(0, 0x00);
 
  for (int ii=0; ii < NUM_ENCODERS; ++ii)
  {
    gDxl.setTxPacketParameter(ii*3+1, 4);
    gDxl.setTxPacketParameter(ii*3+2, gDxlInfo[ii].id);
    gDxl.setTxPacketParameter(ii*3+3, ADDR_PRESENT_POSITION_L);
  }

  int error;
  do
  {
    gDxl.txPacket(BROADCAST_ID, INST_BULK_READ, NUM_ENCODERS*3+1);

    error = 0;
    for (int ii=0; ii < NUM_ENCODERS; ++ii)
    {
      if (gDxl.rxPacket(6+4) != 6+4)
      {
        error = 1;
        break;
      }

      gDxlInfo[ii].pos = DXL_MAKEWORD(gDxl.getRxPacketParameter(0),gDxl.getRxPacketParameter(1));
      gDxlInfo[ii].speed = DXL_MAKEWORD(gDxl.getRxPacketParameter(2),gDxl.getRxPacketParameter(3));
      //gDxlInfo[ii].torque = DXL_MAKEWORD(gDxl.getRxPacketParameter(4),gDxl.getRxPacketParameter(5));
      //gDxlInfo[ii].voltage = gDxl.getRxPacketParameter(6);
      //gDxlInfo[ii].temp = gDxl.getRxPacketParameter(7);
    }

    if (error)
      digitalWrite(PIN_LED_ERROR, 0);

  } while (error);

  digitalWrite(PIN_LED_ERROR, 1);
}

void writeControls()
{
  gDxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);
  gDxl.pushByte(ADDR_GOAL_POSITION_L);
  gDxl.pushByte(6);
  for (int ii=0; ii < NUM_ACTUATORS; ++ii)
  {
    gDxl.pushByte(gDxlInfo[ii].id);

    // Goal position
    gDxl.pushParam(gDxlInfo[ii].torque<0?gDxlInfo[ii].cw_limit:gDxlInfo[ii].ccw_limit);

    // Goal velocity (maximum)
    gDxl.pushParam(0); 

    // Goal torque
    gDxl.pushParam(abs(gDxlInfo[ii].torque));
  }
  gDxl.flushPacket();
}

int readControls()
{
  // Send status to PC
  gPC.writeRaw(0xFF);
  gPC.writeRaw(gMode);
  for (int ii=0; ii < NUM_ENCODERS; ++ii)
  {
    if (ii==diArm) continue;
    gPC.writeRaw(gDxlInfo[ii].pos&127);
    gPC.writeRaw((gDxlInfo[ii].pos>>7)&127);
    gPC.writeRaw(gDxlInfo[ii].speed&127);
    gPC.writeRaw((gDxlInfo[ii].speed>>7)&127);
  }

  // Read action from PC. Time out on control step time.
  while (!gPC.available() || gPC.readRaw() != 0xFF)
    if (gStep)
      return 1;

  for (int ii=0; ii < NUM_ACTUATORS+1; ++ii)
  {
    while (!gPC.available())
      if (gStep)
        return 1;
        
    unsigned char r = gPC.readRaw();
    if (ii)
    {
      SerialUSB.println("Read");
      int t = (r&127)*((r&128)?-1:1);
      SerialUSB.println(t);
      gDxlInfo[ii-1].torque = t*8;
      SerialUSB.println(gDxlInfo[ii-1].torque);
    }
    else if (r)
    {
      standup();
      gMode = MODE_RIGHTING;
    }
  }

  return 0;
}

void loop() {
  if (gMode != MODE_IDLE && digitalRead(PIN_BUTTON_RESET) == 1)
  {
    // Go limp
    gDxl.torqueDisable(BROADCAST_ID);
    gMode = MODE_IDLE;
  }
  
  digitalWrite(PIN_LED_CONTROL, !(gMode == MODE_PARKING || gMode == MODE_CONTROL));
  digitalWrite(PIN_LED_ARM_MOVING, !(gMode == MODE_RIGHTING || gMode == MODE_PARKING));
  
  readState();
  int validCommand = !readControls();  
  
  switch (gMode)
  {
    case MODE_IDLE:
      // Wait for user to start robot
      if (digitalRead(PIN_BUTTON_START) == 1)
      {
        standup();
        gMode = MODE_RIGHTING;
      }
      break;
    case MODE_RIGHTING:
      // Wait for arm to point downwards
      if (abs(gDxlInfo[diArm].pos - gDxlInfo[diArm].cw_limit) < ALLOWED_POSITION_ERROR)
        gMode = MODE_AWAIT_CONTROL;
      break;
    case MODE_AWAIT_CONTROL:
      // Wait for PC to send control data
      if (validCommand)
      {
         // Move arm to park position
         gDxl.goalPosition(gDxlInfo[diArm].id, gDxlInfo[diArm].ccw_limit);
         writeControls();
         gDxl.torqueEnable(BROADCAST_ID);
         gMode = MODE_PARKING;
       }
      break;
    case MODE_PARKING:
      // Wait for arm to reach parking position
      if (abs(gDxlInfo[diArm].pos - gDxlInfo[diArm].ccw_limit) < ALLOWED_POSITION_ERROR)
      {
        // Disable torque (arm is held by magnet)
        gDxl.torqueDisable(gDxlInfo[diArm].id);
        gMode = MODE_CONTROL;
      }
      // fallthrough
    case MODE_CONTROL:
      // Regular control mode
      if (abs(gDxlInfo[diEncoderRoll].pos) > FALL_ANGLE)
      {
        // Robot fell
        standup();
        gMode = MODE_RIGHTING;
      }
      else if (validCommand)
        writeControls();
      break;
  }

  while (!gStep);
  gStep = 0;
}

void stepHandler()
{
  gStep = 1;
}
