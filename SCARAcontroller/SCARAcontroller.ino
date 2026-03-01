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


// Global variables
String robotCmd = "";
bool stringComplete{0};
bool stringError{0};
bool cmdRequest{0};
const int numJoints = 4;
bool motorEnable{0};
bool indexMotor[numJoints] = {0,0,0,0};


// Robot variables
long qPos[numJoints]  = {0,0,0,0};
long qdot[numJoints]  = {0,0,0,0};
long qDist[numJoints] = {0,0,0,0};
moveMode mode = moveMode::moveIdle;

unsigned long startTime;
unsigned long loopTime;

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

  // Change postive direction
  mtr2.setPinsInverted(true,false,false);
  mtr3.setPinsInverted(true,false,false);

  
  for(auto mo:mtrs)
  {
    mo->setMaxSpeed(2500);
    mo->setAcceleration(2500);

  }

  // Set motor enable pin
  pinMode(mtrEnablePin,OUTPUT);
  digitalWrite(mtrEnablePin,HIGH);
}

void loop() 
{
  // 1. Prompt user for a robot command
  if(!cmdRequest)
  {
    Serial.print("READY. Current mode.");
    Serial.println((int)mode);
    cmdRequest = true;
  }
  // 2. Check if new serial input;
  serialEvent();
  
  if(stringComplete)
  {
    startTime = micros();
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

   //4. Motion Logic
   switch(mode)
   {
      case moveMode::moveAbsolute:
        for(auto i = 0; i < numJoints; ++i)
        {
          mtrBasics[i]->moveAbsolute(*mtrs[i],*mtrPars[i],qPos[i],5);
        }
        if(allMotorsIdle()) mode = moveMode::moveIdle;
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
            mtrBasics[i]->mtrCal(*mtrs[i],*mtrPars[i]);
          }
        }
          // mtrBasics[0]->mtrCal(*mtrs[0],*mtrPars[0]);
          // mtrBasics[1]->mtrCal(*mtrs[1],*mtrPars[1]);
          // mtrBasics[2]->mtrCal(*mtrs[2],*mtrPars[2]);

        // Reset Motor Index Enable
        for(auto i = 0; i < numJoints; i++)
        {
          indexMotor[i] = 0;
        }
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
    }
    else if(token.startsWith("motorsoff"))
    {
      motorEnable = 0;
    }
    else if(token.startsWith("home"))
    {
      if(!modeSet)
      {
        mode = moveMode::moveHoming;
        modeSet = true;
        // for(auto i = 0; i < numJoints; ++i)
        // {
        //   mtrBasics[i]->isCalibrating = true;
        // }
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