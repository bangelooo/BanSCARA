#include "ET.h"

// Constructor
ET::ET(trnType E,float eta)
    : type{E},eta{eta} // Initialization list
    {}
// Destructor
ET::~ET() = default;

void ET::createMatrix(HT T)
{
    // Initialize identity matrix
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j ++)
        {
            T[i][j] = (i==j) ? 1.0 : 0.0; // Ternary operator syntax (condition ? valueIfTrue : valueIfFalse)
        }
    }
    double ct = cos(eta);
    double st = sin(eta);
    // Update matrix to correct type
     switch(type)
    {
        case Rx:
            T[1][1] = ct;
            T[1][2] = -st;
            T[2][1] = st;
            T[2][2] = ct;
            break;
        case Ry:
            T[0][0] = ct;
            T[0][2] = st;
            T[2][0] = -st;
            T[2][2] = ct;
            break;
        case Rz:
            T[0][0] = ct;
            T[0][1] = -st;
            T[1][0] = st;
            T[1][1] = ct;
            break;
        case Tx:
            T[0][3] = eta;
            break;
        case Ty:
            T[1][3] = eta;
            break;
        case Tz:
            T[2][3] = eta;
            break;
    }
}
void ET::print(const HT T)
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