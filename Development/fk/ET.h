#ifndef _ET_H_
#define _ET_H_

#include <iostream>
#include <math.h>

typedef double HT[4][4];

class ET
{
    public:
        enum trnType {Rx,Ry,Rz,Tx,Ty,Tz};
        trnType type;    
        float eta;
    // Constructor
    ET(trnType E, float eta);
    // Destructor
    ~ET();
    
    void createMatrix(HT T);
    void print(const HT T);
};

#endif