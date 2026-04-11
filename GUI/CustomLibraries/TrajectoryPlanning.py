import numpy as np
import matplotlib.pyplot as plt

class CubicPoly:
    def __init__(self,startTime,endTime,startPos,endPos,startVel,endVel):
        self.startTime = startTime
        self.endTime = endTime
        self.startPos = startPos
        self.endPos = endPos
        self.startVel = startVel
        self.endVel = endVel
        self.coeff = self.findCoeff()
    
    def findCoeff(self):
        C = np.array([[self.startPos],
                      [self.startVel],
                      [self.endPos],
                      [self.endVel]])
        A = np.array([[1, self.startTime, self.startTime**2 , self.startTime**3],
                      [0, 1             , 2 * self.startTime, 3 * self.startTime**2],
                      [1, self.endTime  , self.endTime**2   , self.endTime**3],
                      [0, 1             , 2 * self.endTime  , 3 * self.endTime**2]])
        Ainv = np.linalg.inv(A)
        B = np.dot(Ainv,C)
        roundB = np.round(B,5)
        return roundB.flatten()
    
    def displayCoeff(self):
        print(f"a0 = {self.coeff[0]}, a1 = {self.coeff[1]}, a2 = {self.coeff[2]}, a3 = {self.coeff[3]}")

    def displayEquations(self):
        print(f"theta(t) = {self.coeff[0]} + {self.coeff[1]}t + {self.coeff[2]}t^2 + {self.coeff[3]}t^3")
        print(f"thetaDot(t) = {self.coeff[1]} + {2 * self.coeff[2]}t + {3 * self.coeff[3]}t^2")
        print(f"thetaDDot(t) = {2*self.coeff[2]} + {6*self.coeff[3]}t")
    
    def calculatePos(self,t):
        pos = self.coeff[0] + self.coeff[1] * t + self.coeff[2] * t**2 + self.coeff[3] * t**3
        return pos
    
    def calculateVel(self,t):
        vel = self.coeff[1] + 2 * (self.coeff[2] * t) + (3 * self.coeff[3] * t**2)
        return vel
    
    def calculateAcc(self,t):
        acc = (2 * self.coeff[2]) + (6 * self.coeff[3] * t)
        return acc
    
    def calculateMotion(self,t):
       pos = self.calculatePos(t)
       vel = self.calculateVel(t)
       acc = self.calculateAcc(t)
       return [pos,vel,acc]

class LinearParaBlend:
    def __init__(self,startTime,endTime,startPos,endPos,blendAcc):
        self.startTime = startTime
        self.endTime = endTime
        self.startPos = startPos
        self.endPos = endPos
        self.blendAcc = blendAcc
        # Methods to calculate blend time, linear velocity, and blend position
        self._findBlendTime()
        self._findLinVel()
        self._findBlendPos()
        self.stepSize = 1000
        self.trajProf = [self.calcPosProfile(),self.calcVelProfile(),self.calcAccProfile()]

    def _findBlendTime(self):
        """
        Returns blend time, t_b
        """
        A = (self.endTime - self.startTime) / 2
        B = ((self.blendAcc)**2) * (self.endTime - self.startTime)**2
        C = 4 * self.blendAcc * (self.endPos - self.startPos)
        D = 2 * self.blendAcc
        temp = A - np.sqrt(B-C)/D
        self.blendTime = round(temp,4)
    
    def _findLinVel(self):
        temp = self.blendAcc * (self.blendTime - self.startTime)
        self.linVel = round(temp,3)
    
    def _findBlendPos(self):
        temp = self.startPos + 0.5 * self.blendAcc * (self.blendTime - self.startTime)**2
        self.blendPos = round(temp,3)
    
    def calculatePos(self,t):
        if (t >= self.startTime) and (t < self.blendTime):
            pos = self.startPos + 0.5 * self.blendAcc * t**2    
        elif (t >= self.blendTime) and (t < (self.endTime - self.blendTime)):
            pos = self.blendPos + self.linVel * (t - self.blendTime)
        elif (t >= (self.endTime - self.blendTime)) and (t <= self.endTime):
            pos = self.endPos - 0.5 * self.blendAcc * (self.endTime - t)**2
        return pos

    def calculateVel(self,t):
        if (t >= self.startTime) and (t < self.blendTime):
            vel = self.blendAcc * t    
        elif (t >= self.blendTime) and (t < (self.endTime - self.blendTime)):
            vel = self.linVel
        elif (t >= (self.endTime - self.blendTime)) and (t <= self.endTime):
            vel = self.blendAcc * (self.endTime - t)
        return vel    
   
    def calculateAcc(self,t):
        if (t >= self.startTime) and (t < self.blendTime):
            acc = self.blendAcc 
        elif (t >= self.blendTime) and (t < (self.endTime - self.blendTime)):
            acc = 0
        elif (t >= (self.endTime - self.blendTime)) and (t <= self.endTime):
            acc = -self.blendAcc
        return acc   
    
    def calcPosProfile(self):
        t = np.linspace(self.startTime,self.endTime,self.stepSize)
        pos = np.zeros(len(t))
        for i,val in enumerate(t):
            pos[i] = self.calculatePos(val)
        return pos

    def calcVelProfile(self):
        t = np.linspace(self.startTime,self.endTime,self.stepSize)
        vel = np.zeros(len(t))
        for i,val in enumerate(t):
            vel[i] = self.calculateVel(val)
        return vel
    
    def calcAccProfile(self):
        t = np.linspace(self.startTime,self.endTime,self.stepSize)
        acc = np.zeros(len(t))
        for i,val in enumerate(t):
            acc[i] = self.calculateAcc(val)
        return acc


    def plotTraj(self):
        t = np.linspace(self.startTime,self.endTime,self.stepSize)
        fig, ax = plt.subplots(3,1)
        ax[0].plot(t,self.trajProf[0])
        ax[1].plot(t,self.trajProf[1])
        ax[2].plot(t,self.trajProf[2])
        plt.tight_layout()
        plt.show()

    def showEquations(self):
        print("\nSegment 1: t_0 <= t <= t_b")
        print("=======================================")
        print(f"theta(t) = {self.startPos} + {0.5 * self.blendAcc}t^2")
        print(f"thetadot(t) = {self.blendAcc}t")
        print(f"thetaddot(t) = {self.blendAcc}")

        print("\nSegment 2: t_b <= t <= (t_f - t_b)")
        print("=======================================")
        print(f"theta(t) = {self.blendPos} + {self.linVel}(t - {self.blendTime})")
        print(f"thetadot(t) = {self.linVel}")
        print(f"thetaddot(t) = 0")

        print("\nSegment 3: (t_f - t_b) <= t <= t_f")
        print("=======================================")
        print(f"theta(t) = {self.endPos} - {0.5 * self.blendAcc}({self.endTime} - t)^2")
        print(f"thetadot(t) = {self.blendAcc}({self.endPos} - t)")
        print(f"thetaddot(t) = {-self.blendAcc}")

class TrapezoidalTrajectory():
    def __init__(self,pStart:float ,pEnd:float,maxSpeed:float,maxAcc:float,mtnRule:str = "1/3"):
        # Defined by user
        self.pStart = pStart
        self.pEnd = pEnd
        self.vMax = abs(maxSpeed)
        self.mtnRulePcn = float(mtnRule[0])/ float(mtnRule[2])
        self.maxAcc = maxAcc
        
        # Class Constants   
        self.stepSize = 1000

        # Determine direction (velocity)
        if(self.pStart >= self.pEnd): # Example: If 10 > 5
            self.vMax *= -1 # Change direction 
        else:
            self.vMax *= 1 # Keep direction
        #print(f"Initial velocity: {self.vMax}")
        # Methods to get parameters needed to generate trajectory

        if (self.pEnd - self.pStart == 0.0):
            self.tTotal = 1.0 # seconds
            self.acc = 0
            self.timeVec = np.linspace(0,self.tTotal,self.stepSize)
            position = np.full(shape=self.stepSize,fill_value=self.pEnd)
            velocity = np.zeros(self.stepSize)
            acceleration = np.zeros(self.stepSize)
            self.trajectory = [position,velocity,acceleration]
            #print("Trajectory constant")

        else:
            self._getTrajParameters()
            self._getBlendPosition()

            # Store Trajectory
            self.trajectory = self.calcTrajProfile()

    # ======== METHODS TO GET TRAJECTORY PARAMETERS =======
    def _getTrajParameters(self):
        totTrav = np.abs(self.pEnd - self.pStart)
        spdMax = np.abs(self.vMax)
        
        # Calculate total time for trapezoidal trajectories
        x = 2 - 2 * self.mtnRulePcn
        self.tTotal = ((2 * totTrav) / spdMax) / x

        #print(f"Initial travel time {self.tTotal}")
        
        # Get accleration time
        self.tAcc = self.mtnRulePcn * self.tTotal
        #print(f"Initial accleration time: {self.tAcc}")
        
        # Check for max acceleration. Update motion time if exceeeded.
        acc = self.vMax / self.tAcc # Keep in mind direction
        #print(f"Initial acceleration: {acc}")

        self.acc = acc # Initial attribute creation
        if abs(acc) > abs(self.maxAcc):
            #print("========== Warning! Exceeded max acceleration. Calculating new trajectory.==========")
            accMag = abs(self.maxAcc)
            
            # Re-calculate total time
            tTotal = np.sqrt(totTrav / ((1 - self.mtnRulePcn) * accMag * self.mtnRulePcn))
            self.tTotal = tTotal
            #print(f"New travel time: {tTotal}")

            # Re-calculate acceleration time
            self.tAcc = self.mtnRulePcn * self.tTotal
            #print(f"New acceleration time: {self.tAcc}")
            if (self.pStart >= self.pEnd):
                self.acc = -1 * accMag
        
            else:
                self.acc = accMag

            #print(f"New Acceleration:{self.acc}")

            # Calculate velocity
            self.vMax = self.acc * self.tAcc

            #print(f"New velocity: {self.vMax}")

        self.timeVec = np.linspace(0,self.tTotal,self.stepSize)
    
    def _getBlendPosition(self):
        self.pBlend = self.pStart + 0.5 * (self.acc) * (self.tAcc)**2

    # ======== METHODS TO CALCULATE TRAJECTORY =======

    def _findTrajectory(self,t):
        if (t >= 0) and (t < self.tAcc):
            pos = self.pStart + 0.5 * self.acc * t**2
            vel = self.acc * t
            acc = self.acc

        elif (t >= self.tAcc) and (t < (self.tTotal - self.tAcc)):
            pos = self.pBlend + self.vMax * (t - self.tAcc)
            vel = self.vMax
            acc = 0 

        elif (t >= (self.tTotal - self.tAcc)) and (t <= self.tTotal):
            pos = self.pEnd - 0.5 * self.acc * (self.tTotal - t)**2
            vel = self.acc * (self.tTotal - t)
            acc = -self.acc
        return [pos,vel,acc]
    
    def calcTrajProfile(self):
        t = self.timeVec
        pos = np.zeros(len(t))
        vel = np.zeros(len(t))
        acc = np.zeros(len(t))
        for i,val in enumerate(t):
            profile = self._findTrajectory(val)

            pos[i] = profile[0]
            vel[i] = profile[1]
            acc[i] = profile[2]
        return [pos,vel,acc]

    # ======= METHODS TO SIMULATE TRAJECTORY IN REAL TIME ======
    def positionRT(self,t):
        if t <=self.tTotal:
            currentPosition = np.interp(t,self.timeVec,self.trajectory[0])
        else:
            currentPosition = self.trajectory[0][-1]
        return currentPosition

    def plotTraj(self):
        t = self.timeVec
        _, ax = plt.subplots(3,1)
        for i in range(3):
            ax[i].plot(t,self.trajectory[i])
            ax[i].grid()
        plt.tight_layout()
        plt.show()
