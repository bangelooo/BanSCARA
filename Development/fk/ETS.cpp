#include "ETS.h"

// Elementary Transform Sequence Class
void ETS::multiply(const HT A, const HT B, HT C)
{
	for(int i = 0; i < 4; i++)
	{
	    for(int j = 0; j < 4; j++)
		{
			C[i][j]= 0;
			for(int k = 0; k < 4; k++)
			{
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
}
//Constructor
ETS::ETS() 
:etCount(0) // Initialization List
{}
// Destructor
ETS::~ETS()
{
    for(int i = 0; i < etCount; i++)
    {
        delete etList[i];
    }
};

// Methods
void ETS::add(ET *et)
{
    if(etCount < 10)
    {
        etList[etCount++] = et; // Same as vector.pushback
    }
}

void ETS::fk(HT T)
{   // Initialize 4 x 4 identity matrix;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            T[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    HT current, next;
    for(int n = 0; n < etCount; n++)
    {
        etList[n]->createMatrix(current);
        multiply(T,current,next);
        // Copy next to T
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
                {
                    T[i][j] = next[i][j];
                }
        }
    }
}
void ETS::printHT(const HT T)
{
    for(int i = 0; i < 4; i++)
    {
	    for(int j = 0; j < 4; j++)
		{
			std::cout << T[i][j] << "\t"; // Replace with print() in Arduino IDE
		}
		std::cout << std::endl;
	}
	
}