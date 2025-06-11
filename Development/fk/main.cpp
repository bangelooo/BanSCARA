#include <iostream>
#include "ET.h"
#include "ETS.h"
/*
This code is for the forward kinematics development for the SCARA robot.
The code will have to be modified to fit the Arduino environment.
*/

int main()
{
    ETS banSCARA;
    
    // Define constants
    float d1 = 10;
    float q1 = M_PI/4;
    float a1 = 300;
    float q2 = 0;
    float a2 = 300;
    float d3 = 0;
    float q4 = 0;

    // Elementary Transform Sequence (Do not change order)
    banSCARA.add(new ET(ET::Tz,d1));
    banSCARA.add(new ET(ET::Rz,q1));
    banSCARA.add(new ET(ET::Tx,a1));
    banSCARA.add(new ET(ET::Tx,a1));
    banSCARA.add(new ET(ET::Rz,q2));
    banSCARA.add(new ET(ET::Tx,a2));
    banSCARA.add(new ET(ET::Tz,d3));
    banSCARA.add(new ET(ET::Rz,q4));
    // Destructor handles raw pointers

    // Homogeneous tranformation matrix from frame {0} to {EE}
    HT T;
    
    // Perform Forward Kinematics
    banSCARA.fk(T);

    // Print the result
    banSCARA.printHT(T);
}

