/*
 The purpose of this code is to use the AccelStepper library to command a NEMA 17 stepper in various scenarios. A TMC2208 stepper driver is used to drive the motor.

 There are three motion types: Move Absolute, Move Relative and Move Velocity.
    NOTE: Move Absolute and Move Relative functions will not stop until reaching desired position. 
 There are two homing method: Home to Flag and Direct Home.
    Home to Flag: Motor will rotate until the homingPin pushButton is pressed.
    Direct Home: Set '0' to current position.
 Cycling: A nested for loop for the following motor sequence: 0 -> 90 -> 180 -> 270 -> 360 -> repeat.
 */

#include <AccelStepper.h>

// Define I/O 
#define stepPin 3 // STEP pin
#define dirPin 2  // DIRECTION pin
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
  axisParam() // No-args constructor
  :targetPosition{0},targetVelocity{0},targetDistance{0}
  {}
  ~axisParam(){} // Destructor
};

// NOTE: DEFINE THE MICROSTEPPING MODE HERE
microStepMode currentStepMode = microStepMode::half;

// Create instance of status Flags
statusFlags stepMotorStatus;
axisParam stepMotorParam;

// AccelStepper instance in drivermode
AccelStepper stepper(AccelStepper::DRIVER,stepPin,dirPin);

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
  stepper.setMaxSpeed(motorRPM(100)); // Function units: steps/sec
  stepper.setAcceleration(1000); // Function units: steps/sec/sec
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
    Serial.print("Command recieved: ");
    Serial.print(command);
    Serial.print("\tAction: ");

    switch(command)
    { case '1': // MoveRelative
        stepMotorParam.targetDistance = 360;
        stepMotorStatus.motionMode = operatingMode::moveRelative;
        stepMotorStatus.moveRequested = true;
        stepper.move(stepToDeg(stepMotorParam.targetDistance));
        Serial.print("Moving a relative distance of ");
        Serial.print(stepMotorParam.targetDistance);
        Serial.println(" degrees.");
        break;

      case '2': // MoveAbsolute
        stepMotorParam.targetPosition = 0;
        stepMotorStatus.motionMode = operatingMode::moveAbsolute;
        stepMotorStatus.moveRequested = true;
        stepper.moveTo(stepToDeg(stepMotorParam.targetPosition));
        Serial.print("Moving to the absolute position: ");
        Serial.print(stepMotorParam.targetPosition);
        Serial.println(" degrees.");
        break;

      case '3': // MoveVelocity
        stepMotorStatus.motionMode = operatingMode::moveVelocity;
        Serial.println("Constant velocity mode active.");
        break;

      case 'h': // HomeToFlag
      case 'H':
        stepMotorStatus.homing = true;
        stepMotorStatus.homeMethod = homingMethod::homeToFlag;
        Serial.println("Motor commisioning to flag in progress.");
        break;

      case 'd': // DirectHome
      case 'D':
        stepMotorStatus.homing = true;
        stepMotorStatus.homeMethod = homingMethod::directHome;
        break;

      case 'c': // Cycling
      case 'C':
        stepMotorStatus.motionMode = operatingMode::moveAbsolute;
        stepMotorStatus.moveRequested = true;
        stepMotorStatus.cycling = true;
        Serial.println("Cycling in progress.");
        break;
      
      case 's': // Stop the motor
      case 'S':
        // Stop command
        stepMotorStatus.motionMode = operatingMode::moveIdle;
        stepper.stop();
        stepMotorStatus.homing = false;
        stepMotorStatus.cycling = false;
        Serial.println("Motor stopped.");
        break;
    }
  }
  
  // Perform Homing if homing flag is active
  if (stepMotorStatus.homing)
  {
    switch (stepMotorStatus.homeMethod)
    {
      case homingMethod::homeToFlag:
        stepper.setSpeed(motorRPM(8));
        stepper.runSpeed();
        if(sensorValue == LOW)
        {
          stepMotorStatus.homing = false;
          stepper.stop();
          stepper.setSpeed(motorRPM(0));
          stepper.setCurrentPosition(0);
          Serial.println("Motor 0 comissioned to sensor.");
        }
        break;
      case homingMethod::directHome:
        stepMotorStatus.homing = false;
        stepper.setCurrentPosition(0);
        Serial.println("Motor 0 comissioned to current position.");
        break;
        
    }
    
  }
  // Motion Logic
  switch(stepMotorStatus.motionMode)
  {
    case operatingMode::moveRelative:
      if(stepMotorStatus.moveRequested && stepper.distanceToGo() !=0)
      {
        stepper.run();
      }
      if(stepper.distanceToGo()==0)
      {
        stepMotorStatus.moveRequested = false;
      }
    case operatingMode::moveAbsolute:
      if(stepMotorStatus.moveRequested)
      {
        if(stepMotorStatus.cycling)
        {
          // Nested for-loop
          for(int i = 0; i < 5; i++)
          {
            for(int j = 0; j <= 4; j++){
              stepper.moveTo(stepToDeg(j * 90));
              stepper.runToPosition();}
              Serial.print("Cycle count: ");
              Serial.println(i+1);
          }
          stepper.moveTo(stepToDeg(0));
          stepper.runToPosition();
          Serial.println("Cycling Complete.");
          stepMotorStatus.cycling = false;

        }else
          stepper.runToPosition();
      }
      break;
    case operatingMode::moveVelocity:
      stepper.setSpeed(motorRPM(90));
      stepper.runSpeed();
    case operatingMode::moveIdle:
    default:
      break;
  }
  if (stepMotorStatus.moveRequested && stepper.distanceToGo()!=0)
  { 
    stepper.run();
  }
  if (stepper.distanceToGo()==0)
  {
    stepMotorStatus.moveRequested = false;
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
