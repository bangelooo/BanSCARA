#ifndef _ETS_H_
#define _ETS_H_

#include "ET.h"

// Elementary Transform Sequence Class
class ETS
{
    private:
    ET* etList[10]; // Pointer to an array of Elementary transforms
    int etCount;

    void multiply(const HT A, const HT B, HT C);
    
    public:
        //Constructor
        ETS();
        // Destructor
        ~ETS();

        // Methods
        void add(ET *et);
        void fk(HT T);
        void printHT(const HT T);
};

#endif