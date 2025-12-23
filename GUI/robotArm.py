import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class robotLink():
    # Constructor
    def __init__(self,alpha,a,d,theta,jointType,dhType,*args):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.jointType = jointType
        self.dhType = dhType

class robotArm():
    # Constructor
    def __init__(self,links):
        # Class attributes
        self.links = links
        self.numLinks = len(links)

        # Check to see that all links are of the same DH type. If not, throw an error.
        dhTypes = {link.dhType for link in links}
        if len(dhTypes) != 1:
            raise ValueError(f"Mixed DH types found: {dhTypes}")

    # Methods
    def calculateHT(self, linkNum, q):
        """
        Returns the homogeneous transform of the specified robot link
        """
        link = self.links[linkNum]
        a = link.a
        alpha = link.alpha

        # Joint motion
        if link.jointType == "revolute":
            theta = link.theta + q
            d = link.d
        elif link.jointType == "prismatic":
            theta = link.theta
            d = link.d + q
        elif link.jointType == "fixed":
            theta = link.theta
            d = link.d
        else:
            raise ValueError("Unknown joint type")

        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        # DH Convention
        if link.dhType == "std":
        # Standard DH
            HT = np.array([
                [ct, -st * ca,  st * sa, a * ct],
                [st,  ct * ca, -ct * sa, a * st],
                [0,        sa,       ca,      d],
                [0,         0,        0,      1]
            ], dtype=float)

        elif link.dhType == "mod":
            # Modified DH
            HT = np.array([
                [ct,      -st,        0,       a],
                [st * ca,  ct * ca, -sa, -sa * d],
                [st * sa,  ct * sa,  ca,  ca * d],
                [0,         0,        0,       1]
            ], dtype=float)
        else:
            raise ValueError("Unknown DH type")

        return HT

    def forwardKin(self, q):
        """
        Returns the homogeneous transform of the end effector
        and a list of transforms to each link frame.
        """
        if len(q) != self.numLinks:
            raise ValueError("Joint vector length does not match number of links")

        T = np.eye(4)
        jointTransforms = []

        for i in range(self.numLinks):
            Ti = self.calculateHT(i, q[i])
            T = T @ Ti
            jointTransforms.append(T.copy())

        self.FK = T
        self.linkPos = jointTransforms

        return T, jointTransforms

    def scaraIK(self,x,y,z,phi,elbowOrient):
        # Extract link information of robot
        L = ["null"]
        d = ["null"]
        for link in self.links:
            L.append(link.a)
            d.append(link.d)

        # CALCULATE D1
        d1 = z - L[1] - d[4]
        
        # Calculate P3X and P3Y
        p3x = x - L[4] * np.cos(phi)
        p3y = y - L[4] * np.sin(phi)

        A = p3x**2 + p3y**2 - L[3]**2 - L[2]**2
        B = 2 * L[2] * L[3]

        # Check if cos(q3) = A/B is within range
        c3 = A/B
        if (c3 >= -1) and (c3 <= 1):
            if elbowOrient == "elbowDown":
                s3 = np.sqrt(1 - c3**2)
            elif elbowOrient == "elbowUp":
                s3 = -np.sqrt(1 - c3**2)
            else:
                return print("Elbow orientation not recognized. Enter 'elbowDown' OR 'elbowUp'")
        else:
            return print(" Desired pose violates workspace")

        # CALCULATE Q3
        q3 = np.arctan2(s3,c3)

        # Terms for finding Q2
        q2Num = L[3] * np.sin(q3)
        q2Den = L[2] + L[3] * np.cos(q3)

        alpha = np.arctan2(p3y,p3x)
        beta = np.arctan2(q2Num,q2Den)

        # CALCULATE Q2
        q2 = alpha - beta

        # CALCULATE Q4
        q4 = phi - q3 - q2

        q2 = float(np.rad2deg(q2))
        q3 = float(np.rad2deg(q3))
        q4 = float(np.rad2deg(q4))
        
        return (d1,q2,q3,q4)
        
    def displayRobot(self):
        x = [0]
        y = [0]
        z = [0]
        for frame in self.linkPos:
            x.append(frame[0,3].copy())
            y.append(frame[1,3].copy())
            z.append(frame[2,3].copy())

        print(x)
        plt.figure()
        plt.plot(x[1:],y[1:],marker = 'o')
        plt.grid(True)
        plt.xlim(-3,3)
        plt.ylim(-2,2)
        plt.show()
    
    def jacob(self):
        pass

    def animateTraj(self,X,phi,elbowOrient,sample):
        xStart,xEnd = X[0][0], X[0][1]
        yStart,yEnd = X[1][0], X[1][1]
        zStart,zEnd = X[2][0], X[2][1]
        phiStart,phiEnd = phi[0],phi[1]
        q1,q2, q3, q4 = [],[],[],[]

        # Create desired trajectory
        xTraj = np.linspace(xStart,xEnd,sample)
        yTraj = np.linspace(yStart,yEnd,sample)
        #yTraj = np.sqrt(250**2 - xTraj**2)
        zTraj = np.linspace(zStart,zEnd,sample)
        phiTraj = np.linspace(phiStart,phiEnd,sample)

        # Calculate the joint trajectories
        for q in range(sample):
            q1Temp, q2Temp, q3Temp , q4Temp = self.scaraIK(xTraj[q],yTraj[q],zTraj[q],phiTraj[q],elbowOrient)
            q1.append(q1Temp)
            q2.append(q2Temp)
            q3.append(q3Temp)
            q4.append(q4Temp)
        
        # Create the jplot
        fig, ax = plt.subplots()
        ax.set_xlim(-300,300)
        ax.set_ylim(-300,300)
        ax.set_aspect('equal')
        ax.grid(True)

        # Line object representing the robot arm
        (armLine,) = ax.plot([],[], marker = 'o')

        for i in range(len(xTraj)):
            ax.scatter(xTraj[i],yTraj[i],color = 'black')
        ax.scatter([xEnd],[yEnd],color = 'red',label = "Target")
        ax.scatter([xStart],[yStart],color = 'blue',label = "Start")

        # Update the function for anmiation
        def update(frame):
            th1 = q1[frame]
            th2 = q2[frame]
            th3 = q3[frame]
            th4 = q4[frame]
            x,y,z = [], [], []

            TH = [th1,th2,th3,th4]
            TH = np.deg2rad(TH)
            FK, jointTransform = self.forwardKin(TH)

            for transform in jointTransform:
                x.append(transform[0,3].copy())
                y.append(transform[1,3].copy())
                z.append(transform[2,3].copy())
    
            armLine.set_data(x,y)
            return armLine,

        ani = FuncAnimation(fig,update,frames = sample,interval = 30,blit = True)
        plt.title("Cartesian Space Trajectory")
        plt.xlabel("X-Axis (mm)")
        plt.ylabel("Y-Axis (mm)")
        plt.show()
