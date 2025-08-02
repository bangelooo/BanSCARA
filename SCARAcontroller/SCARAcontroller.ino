#include "motorBasic.h"

// Define I/O
#define stepPin 3   // STEP pin for motor 1
#define dirPin 2    // DIRECTION pin for motor 1

#define stepPin2 9  // STEP pin for motor 2
#define dirPin2  8  // DIRECTION pin for motor 1

#define stepPin3 11
#define dirPin3 10

#define ms1Pin 7  // Microstepping 1
#define ms2Pin 6  // Microstepping 2

// Create AccelStepper instance in driverMode
AccelStepper mtr(AccelStepper::DRIVER,stepPin,dirPin);
AccelStepper mtr2(AccelStepper::DRIVER,stepPin2,dirPin2);
AccelStepper mtr3(AccelStepper::DRIVER,stepPin3,dirPin3);
AccelStepper *mtrs[]={&mtr,&mtr2,&mtr3};

// Create instance of axis parameters
axisPar mtrPar1(1,19,microStepMode::half);
axisPar mtrPar2(2,20,microStepMode::half);
axisPar mtrPar3(3,20,microStepMode::half);
axisPar *mtrPars[] = {&mtrPar1,&mtrPar2,&mtrPar3}; 

// Create instance of motorBasic
motorBasic mtrBasic1;
motorBasic mtrBasic2;
motorBasic mtrBasic3;
motorBasic *mtrBasics[] = {&mtrBasic1,&mtrBasic2,&mtrBasic3};



// Global variables
String robotCmd = "";
bool stringComplete{0};
bool stringError{0};
bool cmdRequest{0};
const int numJoints = 3;


// Robot variables
long qPos[numJoints]  = {0,0,0};
long qdot[numJoints]  = {0,0,0};
long qDist[numJoints] = {0,0,0};
moveMode mode = moveMode::moveIdle;

void setup() 
{
  Serial.begin(9600); // Begin serial communication with a baud rate of 9600
  while(!Serial){}
  delay(1000);       // Give the serial monitor time to initialize 
  
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);

   /*  MS1   MS2   STEP
      H     L     1/2
      L     H     1/4
      L     L     1/8
      H     H     1/16
  */

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

  // Set micro-stepping pins as outputs
  for(auto mo:mtrs)
  {
    mo->setMaxSpeed(1500);
    mo->setAcceleration(2000);
  }

}

void loop() 
{
  // 1. Prompt user for a robot command
  if(!cmdRequest)
  {
    Serial.println("Please enter a robot command.");
    cmdRequest = true;
  }
  // 2. Check if new serial input;
  serialEvent();
  
  if(stringComplete)
  {
    Serial.print("Command issued: ");
    Serial.println(robotCmd);
    // 3. Parse Robot Command and set parameters
    parseRbtCmd(robotCmd);
    if(!stringError)
    {
      displayParameter();
    }
    // Reset flags
    robotCmd = "";
    stringComplete = false;
    cmdRequest = false;
    stringError = false;
    for(auto i =0; i < numJoints; ++i)
    {
      mtrBasics[i]->moveRequested = false;
    }
    
  }
    //Serial.print(static_cast<int>(mode));

   //4. Motion Logic
   switch(mode)
   {
      case moveMode::moveAbsolute:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->moveAbsolute(*mtrs[i],*mtrPars[i],qPos[i],5);
        }
        break;
      case moveMode::moveRelative:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->moveRelative(*mtrs[i],*mtrPars[i],qDist[i],5);
        }
        break;
      case moveMode::moveStopping:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->stopMotor(*mtrs[i],*mtrPars[i]);
        }
        break;
      case moveMode::moveHoming:
          mtrBasics[0]->mtrCal(*mtrs[0],*mtrPars[0]);
          mtrBasics[1]->mtrCal(*mtrs[1],*mtrPars[1]);
          mtrBasics[2]->mtrCal(*mtrs[2],*mtrPars[2]);
        break;
   }
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
    

    if(token.startsWith("home"))
    {
      if(!modeSet)
      {
        mode = moveMode::moveHoming;
        modeSet = true;
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->isCalibrating = true;
        }
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
          case moveMode::moveRelative:
            qDist[1] = token.substring(3).toFloat();
            mtrBasic2.qSets = true;
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
          case moveMode::moveRelative:
            qDist[2] = token.substring(3).toFloat();
            mtrBasic3.qSets = true;
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
  }

}