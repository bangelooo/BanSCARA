#include "motorBasic.h"

// Constructor
motorBasic::motorBasic()
: isCalibrating{false}, moveRequested{false},qSets{false},relMoveDone{true}
{}
// Destructor
motorBasic::~motorBasic(){};

// Methods
// ========================= ABSOLUTE MOVE =========================
void motorBasic::moveAbsolute(AccelStepper &mtr, axisPar &mtrPar, float position, float speed)
{
  if(isCalibrating)
  {
    mtrPar.qPosDes = mtrPar.degToStep(position);
    mtr.setCurrentPosition(mtrPar.qPosDes);
    isCalibrating = false;
  }else
  {
    if(!moveRequested)
    {
      mtrPar.qPosDes = mtrPar.degToStep(position);
      mtr.moveTo(mtrPar.qPosDes);
      moveRequested = true;
    }
    if(moveRequested && mtr.distanceToGo() !=0)
    {
      mtr.run();
    }
    
    if(moveRequested && mtr.distanceToGo() == 0)
    {
    moveRequested = false;
    }

  }
}
// ========================= RELATIVE MOVE =========================
void motorBasic::moveRelative(AccelStepper &mtr, axisPar &mtrPar, float distance, float speed)
{

  if(!moveRequested)
  {
    mtrPar.qDistDes = mtrPar.degToStep(distance);    
    mtr.move(mtrPar.qDistDes);
    moveRequested = true;
  }

  if(moveRequested && mtr.distanceToGo() != 0)
  {
    mtr.run();
  }

  if(moveRequested && mtr.distanceToGo() == 0)
  {
    mtr.setCurrentPosition(mtr.currentPosition());
    mtrPar.qDistDes = 0;
  }

}
// ========================= VELOCITY MOVE =========================
void motorBasic::moveVelocity(AccelStepper &mtr, axisPar &mtrPar,float speed,float test)
{
  mtrPar.qDotDes = mtrPar.rpmToStepsPerSec(speed);
  mtr.setSpeed(mtrPar.qDotDes);
  mtr.runSpeed();

}

void motorBasic::stopMotor(AccelStepper &mtr, axisPar &mtrPar)
{
  // Stop motor if running a relative or absolute move
  mtr.stop();

  // Stop motor if running a velocity move
  mtr.setSpeed(0);
  mtr.runSpeed();

  // Sync position
  long steps = mtr.currentPosition();

  mtr.setCurrentPosition(steps);
  mtr.moveTo(steps);

  // Update motor parameters
  mtrPar.qDotDes = 0;
  mtrPar.qDistDes = 0;
  mtrPar.qPosDes = mtrPar.stepToDeg(steps);

  // Reset internal flags
  moveRequested = false;
  isCalibrating = false;
  qSets = false;
  
}

void motorBasic::mtrCal(AccelStepper &mtr, axisPar& mtrPar)
{
  mtrPar.qPosDes = 0;
  mtr.setCurrentPosition(mtrPar.qPosDes);
  isCalibrating = false;
}