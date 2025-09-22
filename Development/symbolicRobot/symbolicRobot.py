import sympy as sp

class symbolicRobot():
    def __init__(self,numJoints):
        self.joints = numJoints
        self.createSymbols()
        self.dhTable = []
        self.HT = []
        
    # Methods
    def createSymbols(self):
        """Create the necessary standard DH parameter symbols for the robot.
        Based on the number of joints the robot has."""
        self.a = sp.symbols(f"L0:{self.joints+1}")
        self.alpha = sp.symbols(f"alpha0:{self.joints+1}")
        self.d = sp.symbols(f"d0:{self.joints+1}")
        self.theta = sp.symbols(f"theta0:{self.joints+1}")        
    
    def retrieveSymbols(self):
        return self.a,self.alpha,self.d,self.theta

    def addDHPar(self,a,alpha,d,theta):
        tempDH = [a,alpha,d,theta]
        self.dhTable.append(tempDH)

    def displayDHtable(self):
        sp.pprint(self.dhTable)

    def stdHT(self,a,alpha,d,theta):
	    # Create variables for cosine and sine
	    ct,st = sp.cos(theta) , sp.sin(theta)
	    ca,sa = sp.cos(alpha) , sp.sin(alpha)
		
	    # Standard Denavit Hartenberg matrix
	    HT = sp.Matrix([
		    [ct, -st * ca,  st * sa, a * ct],
		    [st,  ct * ca, -ct * sa, a * st],
		    [0,        sa,       ca,      d],
		    [0,         0,        0,      1]
		    ])
	    return HT

    def extractRot(self,setNum):
        HTset = self.HT
        R = HTset[setNum-1][0:3,0:3]
        return R

    def extractPos(self,setNum):
        HTset = self.HT
        P = HTset[setNum - 1][0:3,3]
        return P