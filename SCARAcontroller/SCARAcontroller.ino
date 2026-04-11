#include "motorBasic.h"

// Define I/O

#define stepPin   3     // STEP pin for motor 1
#define dirPin    2     // DIRECTION pin for motor 1

#define stepPin2  9     // STEP pin for motor 2
#define dirPin2   8     // DIRECTION pin for motor 2

#define stepPin3  11    // STEP pin for motor 3
#define dirPin3   10    // DIRECTION pin for motor 3

#define stepPin4  13    // STEP pin for motor 4
#define dirPin4   12    //  DIRECTION pin for motor 4

#define ms1Pin 7  // Microstepping 1
#define ms2Pin 6  // Microstepping 2

#define mtrEnablePin  4

#define eeEnablePin 5

// Create AccelStepper instance in driverMode
AccelStepper mtr(AccelStepper::DRIVER,stepPin,dirPin);
AccelStepper mtr2(AccelStepper::DRIVER,stepPin2,dirPin2);
AccelStepper mtr3(AccelStepper::DRIVER,stepPin3,dirPin3);
AccelStepper mtr4(AccelStepper::DRIVER,stepPin4,dirPin4);
AccelStepper *mtrs[]={&mtr,&mtr2,&mtr3,&mtr4};
 
// Create instance of axis parameters
microStepMode stepMode = microStepMode::half;
axisPar mtrPar1(1,19,stepMode);
axisPar mtrPar2(2,20,stepMode);
axisPar mtrPar3(3,20,stepMode);
axisPar mtrPar4(4,76,stepMode);

axisPar *mtrPars[] = {&mtrPar1,&mtrPar2,&mtrPar3,&mtrPar4}; 

// Create instance of motorBasic
motorBasic mtrBasic1;
motorBasic mtrBasic2;
motorBasic mtrBasic3;
motorBasic mtrBasic4;

motorBasic *mtrBasics[] = {&mtrBasic1,&mtrBasic2,&mtrBasic3,&mtrBasic4};

// Create Enum for Machine State
enum class MachineState
{
  IDLE = 1,
  BUSY = 2,
  ERROR = 3,
  RESETTING = 4,
  ABORTING = 5,
};

// Global variables
  // Variables for command handling
String robotCmd = "";
bool stringComplete{0};
bool stringError{0};
bool cmdRequest{0};

  // Variables for robot state
const int numJoints = 4;

bool indexMotor[numJoints] = {0,0,0,0};

int commandID{0};

  // Robot variables for tracking movement
long qPos[numJoints]  = {0,0,0,0};
long qdot[numJoints]  = {0,0,0,0};
long qDist[numJoints] = {0,0,0,0};
long qHome[numJoints] = {0,0,0,0};

  // Robot variables for operating mode and machine state
moveMode mode = moveMode::moveIdle;
MachineState state = MachineState::IDLE;

  // Robot variables for I/O
bool motorEnable{0};
bool solenoidTrigger = 0;


unsigned long startTime;
unsigned long loopTime;

void setup() 
{
  Serial.begin(9600); // Begin serial communication with a baud rate of 9600
  while(!Serial){}
  delay(1000);       // Give the serial monitor time to initialize 
  
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);

  // Set EE Enable Pin
  pinMode(eeEnablePin,OUTPUT);
  digitalWrite(eeEnablePin,LOW);

   /*  MS1   MS2   STEP
      H     L     1/2
      L     H     1/4
      L     L     1/8
      H     H     1/16
  */
// Set micro-stepping pins as outputs
  switch(mtrPar1.microMode)
  { case microStepMode::half:
      digitalWrite(ms1Pin,HIGH);
      digitalWrite(ms2Pin,LOW); 
      break;
    case microStepMode::quarter:
      digitalWrite(ms1Pin,LOW);
      digitalWrite(ms2Pin,HIGH);
      break;
    case microStepMode::eigth:
      digitalWrite(ms1Pin,LOW);
      digitalWrite(ms2Pin,LOW);
      break;
    case microStepMode::sixteenth:
      digitalWrite(ms1Pin,HIGH);
      digitalWrite(ms2Pin,HIGH);
      break;
  }

  // Change positive direction
  mtr2.setPinsInverted(true,false,false);
  mtr3.setPinsInverted(true,false,false);

  
  for(auto mo:mtrs) // Range Based For Loop
  {
    mo->setMaxSpeed(2500);
    mo->setAcceleration(2500);
  }

  // Set motor enable pin
  pinMode(mtrEnablePin,OUTPUT);
  digitalWrite(mtrEnablePin,HIGH);

  //displayModeSettings();

  /*
  Serial.println("Robot Ready...");
  Serial.print("Machine State: ");
  Serial.println((int)state);
  Serial.print("Operating Mode: ");
  Serial.println((int)mode);
  */
  
}

void resetStringCmd()
{
  robotCmd = "";
  stringComplete = false;
  cmdRequest = false;
  stringError = false;
}

void loop() 
{
  // 1. Prompt user for a robot command
  if(!cmdRequest)
  {
    cmdRequest = true;
  }
  // 2. Check if new serial input;
  serialEvent();
  
  if(stringComplete)
  {
    if (robotCmd.length() == 0)
    {
      resetStringCmd();
      Serial.println("NACK");  // or "EMPTY"
      return;
    }

    if(state == MachineState::IDLE)
    {
      Serial.println("ACK");
      state = MachineState::BUSY;
    }
    else if(robotCmd == "STOP")
    {
      // Allow command to be parsed.
    }
    else
    {
      resetStringCmd();
      Serial.println("NACK");
      return;
    }
  
    //Serial.print("Command issued: ");
    //Serial.println(robotCmd);

    // 3. Parse Robot Command and set parameters
    parseRbtCmd(robotCmd);

    // if(!stringError)
    // {
    //   displayParameter();
    // }
    // Reset flags
    resetStringCmd();

    for(auto i =0; i < numJoints; ++i)
    {
      mtrBasics[i]->moveRequested = false;
    }
    loopTime = micros() - startTime;

    // Serial.print("Parse time (microseconds): ");
    // Serial.println(loopTime);
  }

  if (motorEnable)
  {
    digitalWrite(mtrEnablePin,LOW);
  } else
  {
    digitalWrite(mtrEnablePin,HIGH);  
  }

  if (solenoidTrigger)
    digitalWrite(eeEnablePin,HIGH);
  else
    digitalWrite(eeEnablePin,LOW);

   //4. Motion Logic
   switch(mode)
   {
      case moveMode::moveAbsolute:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->moveAbsolute(*mtrs[i],*mtrPars[i],qPos[i],5);
        }
        if(allMotorsIdle()) 
          {
          mode = moveMode::moveIdle;
          state = MachineState::IDLE;
          Serial.println("SUCCESS");
          }
        break;
      
      case moveMode::moveRelative:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->moveRelative(*mtrs[i],*mtrPars[i],qDist[i],5);
        }
        if(allMotorsIdle()) 
        {
          for(auto i = 0; i < numJoints; ++i)
          {
            qPos[i] = mtrPars[i]->stepToDeg(mtrs[i]->currentPosition());
            qDist[i] = 0;
            mtrBasics[i]->moveRequested = false;
          }
          mode = moveMode::moveIdle;
          state = MachineState::IDLE;
          Serial.println("SUCCESS");
        }
        break;
      
      case moveMode::moveVelocity:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->moveVelocity(*mtrs[i], *mtrPars[i], qdot[i], 0);

        }
        break;

      case moveMode::moveStopping:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->stopMotor(*mtrs[i],*mtrPars[i]);
        }
        for(auto i = 0; i < numJoints; ++i)
        { 
          long steps = mtrs[i]->currentPosition();
          qPos[i] = mtrPars[i]->stepToDeg(steps);
          mtrPars[i]->qPosDes = qPos[i];
        }
        if(allMotorsIdle()) mode = moveMode::moveIdle;
        break;

      case moveMode::moveHoming:
        for(auto i = 0; i < numJoints; i++)
        {
          if (indexMotor[i] == 1)
          {
            mtrBasics[i]->mtrCal(*mtrs[i],*mtrPars[i],qHome[i]);
          }
        }

        // Reset Motor Index Enable
        for(auto i = 0; i < numJoints; i++)
        {
          indexMotor[i] = 0;
        }
        mode = moveMode::moveIdle;
        state = MachineState::IDLE;
        Serial.println("SUCCESS");
        break;
      default:
        break;
   }
} 

bool allMotorsIdle() {
  for(auto i = 0; i < numJoints; ++i) {
    if(mtrs[i]->distanceToGo() != 0) return false;
  }
  return true;
}

// Non-blocking serial input
void serialEvent()
{
  while(Serial.available())
  {
    char inChar = (char)Serial.read();
    if(inChar =='\n')
    {
      stringComplete = true;
    }else
    {
      robotCmd += inChar;
    }
  }
}

// Parse 
void parseRbtCmd(String cmd)
{
  // Remove all white space, \n and \r
  cmd.trim();

  int fromIndex = 0;
  int nextComma{};
  bool modeSet{0};

  while(fromIndex < cmd.length())
  {
    // Find first comma
    nextComma = cmd.indexOf(',',fromIndex);

    // No comma condition
    if(nextComma == -1)
      nextComma = cmd.length();

    // Create a substring from beginning index to first comma found
    String token = cmd.substring(fromIndex,nextComma);
    token.toLowerCase();
    token.trim();

    if(token.startsWith("motorson"))
    {
      motorEnable = 1;
      state = MachineState::IDLE;
      delay(1000);
      Serial.println("SUCCESS");
    }
    else if(token.startsWith("motorsoff"))
    {
      motorEnable = 0;
      state = MachineState::IDLE;
      delay(1000);
      Serial.println("SUCCESS");
    }
    else if(token.startsWith("solenoidon"))
    {
      solenoidTrigger = 1;
      state = MachineState::IDLE;
      delay(1000);
      Serial.println("SUCCESS");
    }
    else if(token.startsWith("solenoidoff"))
    {
      solenoidTrigger = 0;
      state = MachineState::IDLE;
      delay(1000);
      Serial.println("SUCCESS");
    }
    else if(token.startsWith("home"))
    {
      if(!modeSet)
      {
        mode = moveMode::moveHoming;
        modeSet = true;
      }
    }
    else if(token.startsWith("direct"))
    {
      for(auto i = 1; i < numJoints; i++)
      {
        mtrPars[i]->homeMethod = homingMethod::directHome;
      }
    }
    else if(token.startsWith("hardstop"))
    {
      for(auto i = 1; i < numJoints; i++)
      {
        mtrPars[i]->homeMethod = homingMethod::hardStop;
      }
    }
    else if(token.startsWith("toflag"))
    {
      for(auto i = 1; i < numJoints; i++)
      {
        mtrPars[i]->homeMethod = homingMethod::homeToFlag;
      }
    }
    // Set operation mode
    else if(token.startsWith("abs"))
    {
      if(!modeSet)
      {
        mode = moveMode::moveAbsolute;
        resetMoveRel();
        modeSet = true;
      }
    } else if(token.startsWith("rel"))
    {
      if(!modeSet)
      {
        mode = moveMode::moveRelative;
        modeSet = true;
      }

    } else if(token.startsWith("vel"))
    {
      if(!modeSet)
      {
        mode = moveMode::moveVelocity;
        resetMoveRel();
        modeSet = true;
      }
    } else if(token.startsWith("q1"))
    {
      if(modeSet)
      {
        switch(mode)
        {
          case moveMode::moveAbsolute:
            qPos[0] = token.substring(3).toFloat();
            break;
          case moveMode::moveRelative:
            qDist[0] = token.substring(3).toFloat();
            mtrBasic1.qSets = true;
            break;
          case moveMode::moveVelocity:
            qdot[0] = token.substring(3).toFloat();
            break;
          case moveMode::moveHoming:
            indexMotor[0] = 1;
            qHome[0] = token.substring(3).toFloat();
            break;
        }
      }
    } else if(token.startsWith("q2"))
    {
      if(modeSet)
      {
        switch(mode)
        {
          case moveMode::moveAbsolute:
            qPos[1] = token.substring(3).toFloat();
            break;
          case moveMode::moveRelative:
            qDist[1] = token.substring(3).toFloat();
            mtrBasic2.qSets = true;
            break;
          case moveMode::moveVelocity:
            qdot[1] = token.substring(3).toFloat();
            break;
          case moveMode::moveHoming:
            indexMotor[1] = 1;
            qHome[1] = token.substring(3).toFloat();
            break;
        }
      }
    }else if(token.startsWith("q3"))
    {
      if(modeSet)
      {
        switch(mode)
        {
          case moveMode::moveAbsolute:
            qPos[2] = token.substring(3).toFloat();
            break;
          case moveMode::moveRelative:
            qDist[2] = token.substring(3).toFloat();
            mtrBasic3.qSets = true;
            break;
          case moveMode::moveVelocity:
            qdot[2] = token.substring(3).toFloat();
            break;
          case moveMode::moveHoming:
            indexMotor[2] = 1;
            qHome[2] = token.substring(3).toFloat();
            break;
        }
      }
    } else if(token.startsWith("q4"))
    {
      if(modeSet)
      {
        switch(mode)
        {
          case moveMode::moveAbsolute:
            qPos[3] = token.substring(3).toFloat();
            break;
          case moveMode::moveRelative:
            qDist[3] = token.substring(3).toFloat();
            mtrBasic4.qSets = true;
            break;
          case moveMode::moveVelocity:
            qdot[3] = token.substring(3).toFloat();
            break;
          case moveMode::moveHoming:
            indexMotor[3] = 1;
            qHome[3] = token.substring(3).toFloat();
            break;
        }
      }
    }
    else if(token.startsWith("stop"))
    {
      mode = moveMode::moveStopping;
    }else
    {
      Serial.println("Unknown command");
      stringError = 1;
    }
    fromIndex = nextComma + 1;
   }
   // Reset to Move Idle so that nothing happens if a moveMode is not set.
   if(!modeSet)
   {
     mode = moveMode::moveIdle;
   }
   // If a relative move command is made but no joints commanded
   for(auto i=0; i < numJoints; ++i)
   {
    if(!mtrBasics[i]->qSets)
    {
      qDist[i]=0;
    }
    mtrBasics[i]->qSets = false;
   }
}


void resetMoveRel()
{
  for(auto i = 0; i < numJoints; ++i)
  {
    qDist[i] = 0;
  }
}

void displayParameter()
{
  Serial.print("Machine State: ");
  switch(state)
  {
    case MachineState::IDLE:
      Serial.print("Idle. \t");
      break;
    case MachineState::BUSY:
      Serial.print("Busy. \t");
      break;
    case MachineState::ERROR:
      Serial.print("Error. \t");
      break;
    case MachineState::RESETTING:
      Serial.print("Resetting. \t");
      break;
    case MachineState::ABORTING:
      Serial.print("Aborting. \t");
      break;
  }
  Serial.print("Operating Mode: ");
  switch(mode)
  {
    case moveMode::moveIdle:
      Serial.print("Idle\t");
      Serial.print("Relative distance setpoint reset.");
      Serial.print("\n");
      break;
    case moveMode::moveAbsolute:
      Serial.print("Absolute");
      Serial.print("\t");
      for(auto i = 0; i < numJoints; ++i)
      {
        Serial.print("Joint ");
        Serial.print(i+1);
        Serial.print(" target position:"); 
        Serial.print(qPos[i]);
        Serial.print("\t");
      }
      Serial.print("\n");
      break;
    case moveMode::moveRelative:
      Serial.print("Relative");
      Serial.print("\t");
      for(auto i = 0; i < numJoints; ++i)
      {
        Serial.print("Joint ");
        Serial.print(i+1);
        Serial.print(" target distance:"); 
        Serial.print(qDist[i]);
        Serial.print("\t");
      }
      Serial.print("\n");
      break;
    case moveMode::moveVelocity:
      Serial.print("Velocity");
      Serial.print("\n");
      break;
    case moveMode::moveHoming:
      Serial.println("Homing");
      break;
  }

}

void displayModeSettings()
{
  Serial.println("===== MACHINE STATES =====");
  Serial.println("1: IDLE");
  Serial.println("2: BUSY");
  Serial.println("3: ERROR");
  Serial.println("4: RESETTING");
  Serial.println("5: ABORTING");

  Serial.println("===== OPERATING(MOVE) MODES =====");
  Serial.println("1: IDLE");
  Serial.println("2: RELATIVE");
  Serial.println("3: ABSOLUTE");
  Serial.println("4: VELOCITY");
  Serial.println("5: STOPPING");
  Serial.println("6: HOMING");

}