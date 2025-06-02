/*
 The purpose of this code is to use the AccelStepper library to command two  NEMA 17 stepper in various scenarios. A TMC2208 stepper driver is used to drive the motor.

 There are three motion types: Move Absolute, Move Relative and Move Velocity.
    NOTE: Move Absolute and Move Relative functions will not stop until reaching desired position. 
 There are two homing method: Home to Flag and Direct Home.
    Home to Flag: Motor will rotate until the homingPin pushButton is pressed.
    Direct Home: Set '0' to current position.
 Cycling: A nested for loop for the following motor sequence: 0 -> 90 -> 180 -> 270 -> 360 -> repeat.
 */

#include <AccelStepper.h>

// Define I/O 
#define stepPin 3 // STEP pin for motor 1
#define dirPin 2  // DIRECTION pin for motor 1

#define stepPin2 8 // STEP pin for motor 2
#define dirPin2 9 // DIRECTION pin for motor 2

#define ms1Pin 7  // Microstepping 1 
#define ms2Pin 6  // Microstepping 2
#define homingPin 4 // Homing Pin

// Function Prototypes
long stepToDeg(long);
long motorRPM(long);
long motorAccel(long);
float askForDistance();
void displayMenu();

// Operating Modes
enum class microStepMode
{ half = 2,
  quarter = 4,
  eigth = 8,
  sixteenth = 16,
};

enum class operatingMode
{ moveIdle,
  moveRelative,
  moveAbsolute,
  moveVelocity
};

enum class homingMethod
{ directHome,
  homeToFlag
};

class statusFlags
{
  public:
    bool homing;
    bool moveRequested;
    bool cycling;
    operatingMode motionMode; // enum
    homingMethod  homeMethod; // enum
    statusFlags() // No-args constructor
    //Initialization list
    :homing{false},moveRequested{false},cycling{false},motionMode{operatingMode::moveIdle},homeMethod{homingMethod::homeToFlag}
    {}
    ~statusFlags(){}  // Destructor
};

class axisParam
{
  public:
    long targetPosition;
    long targetVelocity;
    long targetDistance;
    int  motorNumber;
  axisParam(int motorNum) // No-args constructor
  :targetPosition{0},targetVelocity{0},targetDistance{0}
  {
    motorNumber = motorNum;
  }
  ~axisParam(){} // Destructor
};

// NOTE: DEFINE THE MICROSTEPPING MODE HERE
microStepMode currentStepMode = microStepMode::half;

// Create instance of status Flags
statusFlags stepMotorStatus;
statusFlags stepMotor2Status;

statusFlags *statusOfMotors[] = {&stepMotorStatus,&stepMotor2Status};

axisParam stepMotorParam(1);
axisParam stepMotor2Param(2);

axisParam *motorParams[] = {&stepMotorParam,&stepMotor2Param};


// AccelStepper instance in drivermode
AccelStepper stepper(AccelStepper::DRIVER,stepPin,dirPin);
AccelStepper stepper2(AccelStepper::DRIVER,stepPin2,dirPin2);

AccelStepper *stepMotors[]={&stepper,&stepper2};


int numMotors = 2;
int mtrIndex = numMotors - 1;

long distance[] = {360,360};
long position[] = {0,0};
int numCycles = 5;

void setup()
{
  Serial.begin(9600); // Begin serial communication with a baud rate of 9600
  while (!Serial) {}
  delay(1000);  // Give the Serial Monitor time to initialize

  // Cofigure pin as input and enable a weak pull-up resistor
  pinMode(homingPin,INPUT_PULLUP); 

  // Set micro-stepping pins as outputs
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);

  /*  MS1   MS2   STEP
      H     L     1/2
      L     H     1/4
      L     L     1/8
      H     H     1/16
  */

  // Set microstepping mode
  switch (currentStepMode)
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
  
  // Define stepper motor parameters
  for(auto mtrs : stepMotors)
  {
    mtrs->setMaxSpeed(motorRPM(200)); // Function units: steps/sec
    mtrs->setAcceleration(1000); 
  }
  
  displayMenu();
}


void loop() 
{ // Read home sensor
  int sensorValue = digitalRead(homingPin);
  // Command the motor
  if(Serial.available())
  { 
    char command = Serial.read(); // Read the first character
    // Ignore newline and carriage return
    if (command == '\n' || command == '\r') 
    {
      return;
    }
    Serial.print("\nCommand recieved: ");
    Serial.print(command);
    Serial.print("\tAction: ");

    switch(command)
    { case '1': // MoveRelative

        for(int i = 0;i < numMotors;i++)
        {
          motorParams[i]->targetDistance = distance[i];
          statusOfMotors[i]->motionMode = operatingMode::moveRelative;
          statusOfMotors[i]->moveRequested = true;
          stepMotors[i]->move(stepToDeg(motorParams[i]->targetDistance));
          Serial.print ("Motor Number: ");
          Serial.print(motorParams[i]->motorNumber);
          Serial.print(". Moving a relative distance of ");
          Serial.print(motorParams[i]->targetDistance);
          Serial.print(" degrees.\t");
        }
        break;

      case '2': // MoveAbsolute
        for(int i = 0; i < numMotors; i++)
        {
          motorParams[i]->targetPosition = position[i];
          statusOfMotors[i]->motionMode = operatingMode::moveAbsolute;
          statusOfMotors[i]->moveRequested = true;
          stepMotors[i]->moveTo(stepToDeg(motorParams[i]->targetPosition));
          Serial.print("Motor Number: ");
          Serial.print(motorParams[i]->motorNumber);
          Serial.print(". Moving to the absolute position: ");
          Serial.print(motorParams[i]->targetPosition);
          Serial.print(" degrees.\t");
          
        }
        break;

      case '3': // MoveVelocity
        for(int i = 0; i < numMotors; i++)
        {
          statusOfMotors[i]->motionMode = operatingMode::moveVelocity;
          Serial.print("Motor Number: ");
          Serial.print(motorParams[i]->motorNumber);
          Serial.print(". Constant velocity mode active. \t\t");
        }
        break;

      case 'h': // HomeToFlag
      case 'H':
        for(int i = 0; i < numMotors; i++)
        {
          statusOfMotors[i]->homing = true;
          statusOfMotors[i]->homeMethod = homingMethod::homeToFlag;
          Serial.print("Motor Number: ");
          Serial.print(motorParams[i]->motorNumber);
          Serial.print(". Motor commisioning to flag in progress.\t");
        }
        break;

      case 'd': // DirectHome
      case 'D':
        for(int i = 0; i < numMotors; i++)
        {
          statusOfMotors[i]->homing = true;
          statusOfMotors[i]->homeMethod = homingMethod::directHome;
        }
        break;

      case 'c': // Cycling
      case 'C':
        for(int i = 0; i < numMotors; i++)
        {
          statusOfMotors[i]->motionMode = operatingMode::moveAbsolute;
          statusOfMotors[i]->moveRequested = true;
          statusOfMotors[i]->cycling = true;
        }
        Serial.println("Cycling in progress.");
        break;
      
      case 's': // Stop the motor
      case 'S':
        for(int i = 0; i < numMotors; i++)
        {
          statusOfMotors[i]->motionMode = operatingMode::moveIdle;
          stepMotors[i]->stop();
          statusOfMotors[i]->homing = false;
          statusOfMotors[i]->cycling = false;
        }
        Serial.println("Motors stopped.");
        break;
    }
  }
  
 
  for(int i = 0; i < numMotors; i++)
  {
    // Perform Homing if homing flag is active
    if(statusOfMotors[i]->homing)
    {
      switch(statusOfMotors[i]->homeMethod)
      {
        case homingMethod::homeToFlag:
          stepMotors[i]->setSpeed(motorRPM(8));
          stepMotors[i]->runSpeed();
          if(sensorValue == LOW)
          {
            Serial.println("");
            statusOfMotors[i]->homing = false;
            stepMotors[i]->stop();
            stepMotors[i]->setSpeed(motorRPM(0));
            stepMotors[i]->setCurrentPosition(0);
            Serial.print("Motor Number: ");
            Serial.print(motorParams[i]->motorNumber);
            Serial.print(". Comissioned to sensor.\t");
          }
          break;
        case homingMethod::directHome:
          statusOfMotors[i]->homing = false;
          stepMotors[i]->setCurrentPosition(0);
          Serial.print("Motor Number: ");
          Serial.print(motorParams[i]->motorNumber);
          Serial.print(". Comissioned to current position.\t\t");
          break;
      }
    }
    // Motion Logic
    switch(statusOfMotors[i]->motionMode)
    {
      case operatingMode::moveRelative:
        if(statusOfMotors[i]->moveRequested && stepMotors[i]->distanceToGo() !=0)
        {
          stepMotors[i]->run();
        }
        if(stepMotors[i]->distanceToGo()==0)
        {
          statusOfMotors[i]->moveRequested = false;
        }
      case operatingMode::moveAbsolute:
        if(statusOfMotors[i]->moveRequested)
        {
          if(statusOfMotors[i]->cycling) // Repeats per number of motors
          { 
            // Declare number of cycles
            for(int j = 0; j < numCycles; j++)
            {
              
              for(int k = 0; k <= 4; k++)
              {
                // For each sequence in the cycle, do not iterate the next position until the motor has reached its position.
                for(int m = 0; m < numMotors; m++)
                {
                  stepMotors[m]->moveTo(stepToDeg(k*90));
                }
                bool motorsBusy;
                do // Set the motorBusy flag to false, unless the distance to go is not 0.
                {
                  motorsBusy = false; 
                  // This for loop is needed to make sure both motors run at the same time
                  for(int n = 0; n < numMotors; n++)
                  {
                    stepMotors[n]->run();
                    if(stepMotors[n]->distanceToGo() !=0)
                    {
                      motorsBusy = true;
                    }
                  }
                }while(motorsBusy);
              }
              Serial.print("Cycle Number: ");
              Serial.println(j + 1);
            }
            Serial.println("Cycling completed.");
            // Set all cycling flags to false
            statusOfMotors[i]->cycling = false;
            statusOfMotors[i+1]->cycling = false; // Need to fix. Setting this because the cycle of 5 repeats twice (outermost for loop)
          } else
            stepMotors[i]->run();
        }
      break;
      case operatingMode::moveVelocity:
        stepMotors[i]->setSpeed(motorRPM(200));
        stepMotors[i]->runSpeed();
        break;
      case operatingMode::moveIdle:
        default:
        break;
    }
  }
}

// Functions
long stepToDeg(long degrees)
{
  return round(degrees / (1.8/ static_cast<int>(currentStepMode)));
};

long motorRPM(long rpm){
  // 1 RPM = 6 degrees per second
  return round(rpm * (6/(1.8 /static_cast<int>(currentStepMode))));
};

long motorAccel(long rpmPerSec){
  return round(rpmPerSec * (6/(1.8 /static_cast<int>(currentStepMode))));
};

float askForDistance(){

  while(Serial.available() == 0){
  }
  float distance = Serial.parseFloat();
  return distance;
}

void displayMenu(){
  Serial.println("===== Stepper Motor Control Menu =====");
  Serial.println("1 - Move Relative");
  Serial.println("2 - Move Absolute");
  Serial.println("3 - Move Velocity");
  Serial.println("H or h: Motor commission to 0 using flag");
  Serial.println("D or d: Motor commision to 0 using direct homing");
  Serial.println("S or s: Stop motor");
  Serial.println("C or c: Cycling Sequence");
}
