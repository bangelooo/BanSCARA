#include <iostream>
#include <math.h>
/*
This code is for the forward kinematics development for the SCARA robot.
The code will have to be modified to fit the Arduino environment.
*/

typedef double HT[4][4]; // Create an alias

// Elementary Transfrom Class
class ET
{
    public:
        enum trnType {Rx,Ry,Rz,Tx,Ty,Tz};
        trnType type;    
        float eta;
    // Constructor
    ET(trnType E, float eta)
    : type{E},eta{eta} // Initialization list
    {}
    // Destructor
    ~ET() = default;
    
    void createMatrix(HT T)
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
    void print(const HT T)
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
};

//Elementary Transform Sequence Class
class ETS
{
    private:
    ET* etList[10]; // Pointer to an array of Elementary transforms
    int etCount;

    void multiply(const HT A, const HT B, HT C)
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
    public:
        //Constructor
        ETS() 
        :etCount(0) // Initialization List
        {}
        // Destructor
        ~ETS()
        {
            for(int i = 0; i < etCount; i++)
            {
                delete etList[i];
            }
        };

        // Methods
        void add(ET *et)
        {
            if(etCount < 10)
            {
                etList[etCount++] = et; // Same as vector.pushback
            }
        }

        void fk(HT T)
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
    void printHT(const HT T)
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
};

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

    // Elementary Transform Sequence
    banSCARA.add(new ET(ET::Tz,d1));
    banSCARA.add(new ET(ET::Rz,q1));
    banSCARA.add(new ET(ET::Tx,a1));
    banSCARA.add(new ET(ET::Tx,a1));
    banSCARA.add(new ET(ET::Rz,q2));
    banSCARA.add(new ET(ET::Tx,a2));
    banSCARA.add(new ET(ET::Tz,d3));
    banSCARA.add(new ET(ET::Rz,q4));
    
    HT T;
    
    // Perform Forward Kinematics
    banSCARA.fk(T);

    // Print the result
    banSCARA.printHT(T);
}

