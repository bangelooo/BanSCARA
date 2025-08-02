#include "motorBasic.h"



// Constructor
motorBasic::motorBasic()
: isCalibrating{false}, moveRequested{false},qSets{false}
{}

// Destructor
motorBasic::~motorBasic(){};

// Methods

void motorBasic::moveRelative(AccelStepper &mtr, axisPar &mtrPar, float distance, float speed)
{
  if(!moveRequested)
  {
    mtrPar.qDistDes = mtrPar.stepToDeg(distance);
    mtr.move(mtrPar.qDistDes);
    moveRequested = true;
  }
  if(mtr.distanceToGo() != 0)
  {
    mtr.run();
  }
}

void motorBasic::moveAbsolute(AccelStepper &mtr, axisPar &mtrPar, float position, float speed)
{
  if(isCalibrating)
  {
    mtrPar.qPosDes = mtrPar.stepToDeg(position);
    mtr.setCurrentPosition(mtrPar.qPosDes);
    isCalibrating = false;
  }else
  {
    if(!moveRequested)
    {
      mtrPar.qPosDes = mtrPar.stepToDeg(position);
      mtr.moveTo(mtrPar.qPosDes);
      moveRequested = true;
    }
    if(moveRequested && mtr.distanceToGo() !=0)
    {
      mtr.run();
    } 
  }
}

void motorBasic::stopMotor(AccelStepper &mtr, axisPar &mtrPar)
{
  mtr.stop();
}

void motorBasic::mtrCal(AccelStepper &mtr, axisPar& mtrPar)
{
  mtrPar.qPosDes = 0;
  mtr.setCurrentPosition(mtrPar.qPosDes);
  isCalibrating = false;
}