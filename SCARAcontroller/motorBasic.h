#ifndef _MOTOR_BASIC_H_
#define _MOTOR_BASIC_H_

#include <AccelStepper.h>
#include "axisPar.h"

class motorBasic
{
  public:
    bool isCalibrating;
    bool moveRequested;
    bool qSets;
  // Constructor Prototype
  motorBasic();
  // Destructor Prototype
  ~motorBasic();

  void moveRelative(AccelStepper&, axisPar&,float,float);
  void moveAbsolute(AccelStepper&, axisPar&,float,float);

  void stopMotor(AccelStepper&, axisPar&);

  void mtrCal(AccelStepper&, axisPar&);

};

#endif