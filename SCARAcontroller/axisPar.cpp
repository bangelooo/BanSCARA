#include "axisPar.h"
#include <Arduino.h> // Needed because of round() method 


// Constructor
axisPar::axisPar(int motorNum, float gearRat, microStepMode micro)
: motorNumber{motorNum},
  gearRatio{gearRat},
  qPosDes{0.0},
  qDotDes{0.0},
  qDistDes{0.0},
  position{0.0},
  velocity{0.0},
  acceleration{0.0},
  homeMethod{homingMethod::homeToFlag},
  motionMode{moveMode::moveIdle},
  microMode{micro} 
{}

// Destructor
axisPar::~axisPar() = default;

// METHODS
  // Conversions
long axisPar::degToStep(long degrees)
{
  return round((degrees*gearRatio)/(1.8/static_cast<int>(microMode)));
}

long axisPar::stepToDeg(long steps)
{
  return round((steps * (1.8/static_cast<int>(microMode)))/gearRatio);
}

long axisPar::rpmToStepsPerSec(long rpm)
{
    // 1 RPM = 6 degrees per second
  return round(gearRatio * rpm * (6/(1.8 /static_cast<int>(microMode))));
}

long axisPar::stepsPerSecToRPM(long stepsPerSec)
{
  return round(stepsPerSec / (gearRatio * (6/(1.8 /static_cast<int>(microMode)))));
}

  // Update motor parameters

  void axisPar::updateVelocity(AccelStepper & motor)
  {
    long velocityInStepsPerSec = motor.speed();
    velocity = stepsPerSecToRPM(velocityInStepsPerSec);
  }


void axisPar::updatePosition(AccelStepper & motor)
{
  long positionInSteps = motor.currentPosition();
  position = stepToDeg(positionInSteps);
}

  // Display Motor Params
void axisPar::showCurrentPosition(AccelStepper& motor)
{
  Serial.print("Motor Position: ");
  Serial.print(position);
  Serial.println(" degrees");
}