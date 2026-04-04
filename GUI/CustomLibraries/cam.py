import numpy as np

class cycloidalMotionCAM:
    def __init__(self,maxRise,baseCircleDiameter,riseAngle,fallAngle,dwellAtRise):
        self.h = maxRise
        self.Db = baseCircleDiameter
        self.betaRise = riseAngle
        self.betaFall = fallAngle
        self.dwellAtRise = dwellAtRise
        self.thetaFall = self.betaRise + self.dwellAtRise
        self.driveMechType = "CAM"
        self._displacementMap()

    def _camDisplacement(self,angle):
        """
        Input: Angel (Degrees)
        Output: Absolute Position
        """
        # RISE
        if (angle >= 0) and (angle <= self.betaRise):
            theta = np.deg2rad(angle)
            beta = np.deg2rad(self.betaRise)

            s = (self.h/ np.pi) * ((np.pi * theta)/(beta) - (0.5 * np.sin((2 * np.pi * theta) / beta)))
            phase = "rise"
        # DWELL AT RISE
        elif (angle > self.betaRise) and (angle <= (self.betaRise + self.dwellAtRise)):
            s = self.h
            phase = "dwellAtRise"
        # FALL
        elif (angle > (self.betaRise + self.dwellAtRise)) and (angle <= (self.betaRise + self.dwellAtRise) + self.betaFall):
            theta = np.deg2rad(angle)
            thetaFall = np.deg2rad(self.thetaFall)
            betaFall = np.deg2rad(self.betaFall)

            tau = (theta - thetaFall) / betaFall  # normalized fall angle

            s = self.h * (1 - (tau - (1 / (2 * np.pi)) * np.sin(2 * np.pi * tau)))
            phase = "fall"
        
        # DWELL AT BASE
        else:
            s = 0
            phase = "dwellAtFall"
        return s,phase
    
    def _displacementMap(self):
        steps = 1000
        angle = np.linspace(0,360,steps)
        y = np.zeros(len(angle))
        riseMap = [[],[]]
        fallMap = [[],[]]
        riseFlag = False
        for index,theta in enumerate(angle):
            yTemp , phase = self._camDisplacement(theta)
            y[index] = yTemp
            if phase == "rise":
                riseMap[0].append(theta)
                riseMap[1].append(yTemp)
            elif phase == "dwellAtRise":
                if not riseFlag:
                    riseMap[0].append(theta)
                    riseMap[1].append(yTemp)
                    riseFlag = True
            elif phase == "fall":
                fallMap[0].append(theta)
                fallMap[1].append(yTemp)
        
        # Create 2 X N array
        yMap = np.vstack((angle,y))

        # Create attributes
        self.yMap = yMap
        self.riseMap = np.array(riseMap)
        self.fallMap = np.array(fallMap)
    
    def findAngle(self,yDesired):
        if (yDesired < 0) or (yDesired > self.h):
            return print(f"Desired value {yDesired} is out of range.")
        else:
            # Extract angle and displacement info from rise map
            rmAngle = self.riseMap[0]
            rmY = self.riseMap[1]

            # Interpolation
            desiredAngle = np.interp(yDesired,rmY,rmAngle)
            
            return desiredAngle
    
    def findAngleRelative(self,yStart,yEnd):
        if (yEnd < 0 or yStart < 0) or (yEnd > self.h or yStart > self.h):
            return print(f"Desired value {yEnd} is out of range")
        else:
            # Extract angle and dispalcement info from rise map
            rmAngle = self.riseMap[0]
            rmY = self.riseMap[1]

            #Interpolation for startAngle
            startAngle = np.interp(yStart,rmY,rmAngle)
            endAngle = np.interp(yEnd,rmY,rmAngle)

            return endAngle - startAngle
    
    def findPosition(self,angle):
        """
        Determine the position (mm) based on the provided angle (degrees)
        """
        position,phase = self._camDisplacement(angle)

        return position
    




