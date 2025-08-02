#include "axisPar.h"
#include <Arduino.h> // Needed because of round() method 

// Constructor
axisPar::axisPar(int motorNum, float gearRat, microStepMode micro)
: motorNumber{motorNum},
  gearRatio{gearRat},
  qPosDes{0.0},
  qDotDes{0.0},
  qDistDes{0.0},
  homeMethod{homingMethod::homeToFlag},
  motionMode{moveMode::moveIdle},
  microMode{micro} 
{}

// Destructor
axisPar::~axisPar() = default;

// Methods
long axisPar::stepToDeg(long degrees)
{
  return round((degrees*gearRatio)/(1.8/static_cast<int>(microMode)));
}

long axisPar::motorRPM(long rpm)
{
    // 1 RPM = 6 degrees per second
  return round(gearRatio*rpm * (6/(1.8 /static_cast<int>(microMode))));
}