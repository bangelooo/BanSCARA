#ifndef _AXIS_PAR_H_
#define _AXIS_PAR_H_

#include <AccelStepper.h>

enum class microStepMode
{
  half = 2,
  quarter = 4,
  eigth = 8,
  sixteenth = 16  
};

enum class moveMode
{
  moveIdle = 1,
  moveRelative = 2,
  moveAbsolute = 3,
  moveVelocity = 4,
  moveStopping = 5,
  moveHoming = 6
};

enum class homingMethod
{
  directHome,
  homeToFlag
};

class axisPar
{
  public:
    int motorNumber;
    float gearRatio;
    long qPosDes;
    long qDotDes;
    long qDistDes;
    long position;
    long velocity;
    long acceleration;
    homingMethod homeMethod; //enum
    moveMode motionMode; // enum
    microStepMode microMode; // enum
    
    // Constructor Prototype
    axisPar(int, float, microStepMode);

    ~axisPar();

    // Methods
    long degToStep(long);
    long stepToDeg(long);
    long rpmToStepsPerSec(long);
    long stepsPerSecToRPM(long);

    // Update Positions
    void updatePosition(AccelStepper &);
    void updateVelocity(AccelStepper &);

    void showCurrentPosition(AccelStepper&);
};

#endif