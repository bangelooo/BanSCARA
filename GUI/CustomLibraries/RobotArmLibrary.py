# Import Python Libraries
import numpy as np
import matplotlib.pyplot as plt
import serial
from enum import Enum
import time
# Import Custom Libraries
from .RobotCommunications import (
    Publisher,
    Subscriber,
    ActionClient,
    ActionServer
    )
from .TrajectoryPlanning import TrapezoidalTrajectory
from .cam import cycloidalMotionCAM

# ============= ROBOT LINK ======================
class RobotLink():
    # Constructor
    def __init__(self,alpha,a,d,theta,jointType,dhType,name:str,driveMechanism:str = "DIRECT",*args):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.jointType = jointType
        self.dhType = dhType
        self.name = name
        self.driveMechanism = driveMechanism
        self.driveMechObj = None

    def addDriveMechansimObj(self,dmObj):
        if self.driveMechanism == dmObj.driveMechType:
            self.driveMechObj = dmObj
        else:
            print("Mechanism type does not match")

# ============= ROBOT ARM =======================
class RobotArm():
    # Constructor
    def __init__(self,links):
        # Class attributes
        self.links = links
        self.numLinks = len(links)
        self.jointPosition = np.zeros(len(links))
        self.pose = self._getPoseDict()
        self.jointTypes = self._getJointType()
        # Check to see that all links are of the same DH type. If not, throw an error.
        dhTypes = {link.dhType for link in links}
        if len(dhTypes) != 1:
            raise ValueError(f"Mixed DH types found: {dhTypes}")

    # Methods
    # ============================================================
    #                       FORWARD KINEMATICS
    # ============================================================
    def _getJointType(self):
        """
        Returns a tuple of type of joints of the robot (base -> EE)
        """
        jointTypeList = []
        for link in self.links:
            jointTypeList.append(link.jointType)
        return tuple(jointTypeList)

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

    def _getPoseDict(self):
        """
        Returns a dictionary of the robot pose.
        Keys: "FK" , "POSITION", "ANGLE (deg)"
        """
        q = self.jointPosition
        FK , _ = self.forwardKin(q)
        r11 = FK[0][0]
        r21 = FK[1][0]
        yaw = np.arctan2(r21,r11)      # Radians 
        poseDict = {
            "FK": FK,
            "POSITION": [FK[0][3],FK[1][3],FK[2][3]],
            "ANGLE": yaw
            }
        return poseDict
    
    def getJoints(self):
        """
        Returns a list of the current joint positions
        """
        return self.jointPosition

    def updateJoints(self,moveType: str,q):
        """
        Updates the joint positions based on the move type.

        :param moveType: "REL" or "ABS"
        :type moveType: str
        :param q: List of joint positions (in radians)
        """
        if moveType == "REL":
            for jIndex,jValue in enumerate(self.jointPosition):
                jValue += q[jIndex]
        elif moveType == "ABS":
            self.jointPosition = q
        self.pose = self._getPoseDict() # Update POSE dictionary of robot arm
    
    # ============================================================
    #                       INVERSE KINEMATICS
    # ============================================================
    def scaraIK(self,x,y,z,phi,elbowOrient):
        '''
        Returns a tuple of the joint angles (in degrees) required to achieve the desired robot pose.
        
        :param x: x-coordinate (mm)
        :param y: y-coorindate (mm)
        :param z: z-coordinate (mm)
        :param phi: yaw-angle about the base frame (radians)
        :param elbowOrient: Elbow orientation (elbowUp / elbowDown)
        '''
        # Extract link information of robot
        L = ["null"]
        d = ["null"]
        for link in self.links:
            L.append(link.a)
            d.append(link.d)

        # CALCULATE D1
        d1 = z - L[1] - d[4]

        #d1 = self.links[0].driveMechObj.findAngle(d1)
        
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

    # ============================================================
    #                       VELOCITY KINEMATICS
    # ============================================================
    def jacob0(self,whichJacobian: str = "J"):
        """
        Returns the Spatial Jacobian (Base Frame).
        Input: "Jv" = Linear portion | "Jw" = "Angular"
        """

        # Linear and angular Jacobian matrices
        Jv = np.zeros((3,self.numLinks))
        Jw = np.zeros((3,self.numLinks))

        # Get forward kinematics of current joint positions
        FK,jointHT = self.forwardKin(self.jointPosition)
        pEE = FK[:3,3] # End Effector Position

        # Multiply rotation matrix by z0 to get z_i
        z = np.array([0.0,0.0,1.0])
        
        # Base Frame HT is an identity matrix
        T00 = np.eye(4)
        R00 = T00[:3,:3]

        # Set rotation matrix and position matrix for loop 
        prevRotMat = R00
        prevPosMat = T00[:3,3]

        # Generate jacobian matrix
        for i,HT in enumerate(jointHT):
            if self.jointTypes[i] == "prismatic":
                Jv[:,i] = prevRotMat @ z
                Jw[:,i] = np.zeros(3)
            elif self.jointTypes[i] == "revolute":
                Jv[:,i] = np.cross(prevRotMat @ z, pEE - prevPosMat)
                Jw[:,i] = prevRotMat @ z

            # Iterate matrix
            prevRotMat = HT[:3,:3]
            prevPosMat = HT[:3,3]
        
        if whichJacobian == "Jv":
            return Jv
        elif whichJacobian == "Jw":
            return Jw
        else:
            J = np.vstack((Jv,Jw))
            return J

    def jacobEE(self,whichJacobian:str = "J"):
        # Extract Rotation Matrix (Base to EE)
        R0E = self.pose["FK"][:3,:3]
        
        # Get Inverse
        RE0 = np.linalg.inv(R0E)

        # Get spatial jacobian (base frame)
        J = self.jacob0("J")
        Jv = J[:3,:]
        Jw = J[3:,:]

        if whichJacobian == "Jv":
            return RE0 @ Jv
        elif whichJacobian == "Jw":
            return RE0 @ Jw
        else:
            return RE0 @ J
        
    def jacobBanSCARA(self):
        """
        Prints the spatial (base frame) and body (ee frame) jacobians.
        Hand calculations.
        """
        L = [0]
        for link in self.links:
            L.append(link.a)
        
        theta = self.jointPosition
        theta = np.insert(theta,0,0)

        JvEE = np.zeros((3,4))

        JvEE[0,0] = 0
        JvEE[0,1] = L[2] * np.sin(theta[3] + theta[4]) + L[3] * np.sin(theta[4])
        JvEE[0,2] = L[3] * np.sin(theta[4])
        JvEE[0,3] = 0

        JvEE[1,0] = 0
        JvEE[1,1] = - (L[2] * np.cos(theta[3] + theta[4]) + L[3] * np.cos(theta[4]) + L[4])
        JvEE[1,2] = - (L[3] * np.cos(theta[4]) + L[4])
        JvEE[1,3] = - L[4]
        
        JvEE[2,0] = - 1
        JvEE[2,1] = 0
        JvEE[2,2] = 0
        JvEE[2,3] = 0        
        FK = self.pose["FK"]
        R0E = FK[:3,:3]
        Jv0 =  R0E @ JvEE

        # Round
        JvEE = np.round(JvEE,2)
        Jv0 = np.round(Jv0,2)

        print(f"Body Jacobian (EE Frame) Using Derivation:\n {JvEE}")
        print(f"Spatial Jacobian(Base Frame) Using Derviation: \n {Jv0}")

    # ============================================================
    #                       ANIMATION 
    # ============================================================

    def animateTraj(self,X,phi,elbowOrient,sample):
        """
        Animate the trajectory (top-view) of the SCARA robot using a desired pose 
        
        :param X: List of start and end positions for (X,Y,Z). ie (0,1,0,2,0,3)
        :param phi: Yaw about world frame
        :param elbowOrient: Elbow Orientation
        """
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

def createBanSCARA():
    # Declare robot link parametrs 
    L1,L2,L3,L4 = 0,191,191,0
    d4 = 0

    # Declare Links using Standard DH paramters
    link1 = RobotLink(0,0,L1,0,"prismatic","std","q1",driveMechanism="CAM")
    link2 = RobotLink(0,L2,0,0,"revolute","std", "q2")
    link3 = RobotLink(0,L3,0,0,"revolute","std", "q3")
    linkEE = RobotLink(-np.pi,L4,d4,0,"revolute","std", "q4")

    # Create CAM for joint 1
    q1CAM = cycloidalMotionCAM(45,25,90,90,180)
    link1.addDriveMechansimObj(q1CAM)
    
    # Group robot links
    robotlinks = [link1,link2,link3,linkEE]

    # Create robot 
    scaraRobot = RobotArm(robotlinks)

    return scaraRobot

# ============ ROBOT END EFFECTOR ===============

class GripType(Enum):
    INTERNAL = 1
    EXTERNAL = 2

class RobotEE():
    def __init__(self,module,numTeethSG,numTeethPG,fingerLength,startingOffsetOfPG):
        # CONSTANT PARAMETERS
        self.MODULE = module
        self.N_TEETH_SUN_GEAR = numTeethSG
        self.N_TEETH_PLANET_GEAR = numTeethPG
        self.FINGER_LENGTH = fingerLength
        self.GEAR_RATIO = self.N_TEETH_PLANET_GEAR/self.N_TEETH_SUN_GEAR
        self.SUN_GEAR_PITCH_RADIUS = self.MODULE * self.N_TEETH_SUN_GEAR / 2
        self.PLANET_GEAR_PITCH_RADIUS = self.MODULE * self.N_TEETH_PLANET_GEAR /2

        self.STARTING_OFFSET_OF_PG = startingOffsetOfPG
        self.MIN_GRIP_DIAMETER = self.findGripDiameterLimits(0)
        self.MAX_GRIP_DIAMETER = self.findGripDiameterLimits(self.determineSunGearAngle(180))

        # VARIABLES
        self.solenoidEngaged = False
        self.sunGearAngle = 0.0
       
        self.currentGripDiameter = self.findGripDiameter(self.sunGearAngle)
        self.gripType = GripType.INTERNAL

    def determineSunGearAngle(self,thetaPG):
        return thetaPG / self.GEAR_RATIO
    
    def determineSunGearTravel(self,startingGripDiameter,endingGripDiameter):
        thetaStartSG = self.findDriveAngle(startingGripDiameter)
        thetaEndSG = self.findDriveAngle(endingGripDiameter)
        return thetaEndSG - thetaStartSG

    def findGripDiameterLimits(self,thetaSGdeg):
        # Does not account for planet gear offset
        r1 = self.SUN_GEAR_PITCH_RADIUS
        r2 = self.PLANET_GEAR_PITCH_RADIUS
        fl = self.FINGER_LENGTH
        thetaPGdeg = - self.GEAR_RATIO * thetaSGdeg
        thetaPGrad = np.deg2rad(thetaPGdeg)
        rSquared = (r1 + r2)**2 + fl**2 - 2 * (r1 + r2) * fl * np.cos(thetaPGrad)
        diameter = np.sqrt(rSquared) * 2
        return diameter
    
    def findGripDiameter(self,thetaSGdeg):
        # Does account for planet gear offset
        r1 = self.SUN_GEAR_PITCH_RADIUS
        r2 = self.PLANET_GEAR_PITCH_RADIUS
        fl = self.FINGER_LENGTH
        thetaPGdeg = - self.GEAR_RATIO * thetaSGdeg + self.STARTING_OFFSET_OF_PG
        thetaPGrad = np.deg2rad(thetaPGdeg)
        rSquared = (r1 + r2)**2 + fl**2 - 2 * (r1 + r2) * fl * np.cos(thetaPGrad)
        diameter = np.sqrt(rSquared) * 2
        return diameter

    def findDriveAngle(self,gripDiameter):
        if gripDiameter > self.MAX_GRIP_DIAMETER or gripDiameter < self.MIN_GRIP_DIAMETER:
            print(f"Grip Diameter Out of Range. Diamter must be between {self.MIN_GRIP_DIAMETER} mm and {self.MAX_GRIP_DIAMETER}mm. ")
            return
        else:
            r1 = self.SUN_GEAR_PITCH_RADIUS
            r2 = self.PLANET_GEAR_PITCH_RADIUS
            fl = self.FINGER_LENGTH

            thetaPGrad = np.arccos(((r1 + r2)**2 + fl**2 - 0.25 * gripDiameter**2)/(2 * (r1 + r2) * fl))

            thetaPGdeg = np.rad2deg(thetaPGrad)
            thetaSGdeg = thetaPGdeg / self.GEAR_RATIO -self.STARTING_OFFSET_OF_PG

            return thetaSGdeg
    
    def activateSolenoid(self):
        self.solenoidEngaged = True
        print("Solenoid is engaged")

    def deactivateSolenoid(self):
        self.solenoidEngaged = False
        print("Solenoid is disengaged.")

    def displayGripDiameterLimits(self):
        print(f"The grip diameters for the end effector are between {self.MIN_GRIP_DIAMETER} mm and {self.MAX_GRIP_DIAMETER} mm. ")

# ============= ROBOT ARM  CONTROLLER=======================
class MachineState(Enum):
    # Modelled after PackML 
    RESETTING = 1
    IDLE = 2
    STARTING = 3
    EXECUTING = 4
    COMPLETING = 5
    COMPLETE = 6
    ABORTING = 7
    ABORTED = 8
    CLEARING = 9
    STOPPING = 10
    STOPPED = 11

    SUSPENDING = 12
    SUSPENDED = 13
    UNSUSPENDING = 14

    HOLDING = 15
    HELD = 16
    UNHOLDING = 17

class OperatingMode(Enum):
    OPERATION = 1
    SIMULATION = 2

class IndexMode(Enum):
    DIRECT = 1
    HARDSTOP = 2
    TOFLAG = 3

class RobotArmController():
    # Contructor
    def __init__(self,robotObject,topicsList,serviceList,actionsList):        # Instantiate robot arm
        self.robot = robotObject   
        self.endEffector = RobotEE(1.5,20,20,25,11.723)
        self.routine = self.defaultRoutine()

        # ======================================
        #   Publishers and Subscribers
        # ======================================
        # Set publishers and subscribers for robot controller
        pubs = ["jState","cState","calState","gripDiameter"]
        subs = []

        # Add topics from topicsList
        self.topics = {}
        
        for topicObj in topicsList:
            self.topics[topicObj.name] = topicObj

        # Create dictionary for publishers and subscribers
        self.publishers = {}

        for key,item in self.topics.items():
            if key in pubs:
                self.publishers[key + "Pub"] = Publisher(item)

        # self.subscribers = {}
        # for key,item in self.topics.items():
        #     if key in subs:
        #         sub = self.subscribers[key + "Sub"] = Subscriber(key)
                
        #         # Subscribe to the topic
        #         self.topics[key].addSubscriber(sub)
        
        # ======================================
        #   Action Servers
        # ======================================
        actionServers = ["jCmd","cCmd","calCmd","gripCmd","releaseCmd"]

        # Create dictionary of Actions and Action Servers
        self.actions = {}

        for actionObj in actionsList:
            self.actions[actionObj.name] = actionObj

        self.actionServers = {}

        for actionObj in actionsList:
            if actionObj.name in actionServers:
                self.actionServers[actionObj.name + "Server"] = ActionServer(actionObj)
        
        # Assign goal handler functions 
        try:
            self.actionServers["jCmdServer"].assignGHF(self.jSpaceGRH)
            self.actionServers["cCmdServer"].assignGHF(self.cSpaceGRH)
            self.actionServers["calCmdServer"].assignGHF(self.indexGRH)
            self.actionServers["gripCmdServer"].assignGHF(self.gripObjGRH)
            self.actionServers["releaseCmdServer"].assignGHF(self.releaseObjGRH)
        except:
            print("There is an action that does not exist")

        # ======================================
        #   Action Clients
        # ======================================
        self.actionClients = {}

        for actionObj in actionsList:
            if actionObj.name in actionServers:
                self.actionClients[actionObj.name + "Client"] = ActionClient(actionObj)

        
        # ======================================
        #   Controller Attributes
        # ======================================   
        # Logic
        self.operatingMode = OperatingMode.SIMULATION
        self.serialConnect = False  
        self.calibrationState = [False for _ in self.robot.links]
        self.motorsOn = False

        # Controller states
        self.robotState = MachineState.IDLE
        self.jointState = self.robot.jointPosition
        self.poseState = self.robot.pose
        self.desiredElbOri = "elbowDown"
        self.indexingMode = IndexMode.DIRECT

        self.gripDiameter = self.endEffector.currentGripDiameter
        self.gripperEngaged = False
        self.gripType = None

        # Attributes for trajectory
        self.trajTargets = {"J-Space": {}, "C-Space": {}}
        self._initalizeTrajTargets()

        self.trajectoryList = [0] * len(self.robot.links)

        # Attributes for tracking commands
        self.cmdID = 0


    # ======================================
    #   Methods for Serial Communication
    # ======================================

    def connectSerial(self):
        if "serialObj" not in self.__dict__:
            try:
                ser = serial.Serial(
                    port = "/dev/cu.usbmodem21101",
                    baudrate = 9600,
                    timeout = 1)
                self.serialObj = ser
                self.serialConnect = True
                msg = "CONNECTED"
                return msg
            except serial.SerialException as e:
                print(f"Unable to connect: {e}")
        else:
            try:
                self.serialObj.open()
                self.serialConnect = True
                msg = "CONNECTED"
                return msg
            except serial.SerialException as e:
                print(f"Port already open: {e}")
    
    def disconnectSerial(self):
        if "serialObj" not in self.__dict__:
            msg = "Not connected. Cannot disconnect"
            return msg
        else:
            self.serialObj.close()
            self.serialConnect = False
            msg = "Successfully disconnected"
            return msg
    
    def sendCommand(self,cmd):
        if self.operatingMode == OperatingMode.OPERATION:
            self.serialObj.write((cmd + '\n').encode('utf-8'))
            while True:
                response = self.serialObj.readline().decode().strip()
                # Ignore blank strings
                if not response:
                    continue
                if response == "ACK":
                    result = self.processResultRequest()
                    print(result)
                    return result
        else:
            print("SUCCESS")

    def routineTest(self):
        while True:
            self.sendCommand("ABS,Q2:90,Q3:-90")
            self.sendCommand("ABS,Q2:0,Q3:0")
            self.sendCommand("ABS,Q2:45,Q3:-45")

    def processResultRequest(self):
        '''
        Blocking function. Process responses from Arduino.
        '''
        while True:
            response = self.serialObj.readline().decode().strip()
            # Ignore blank strings
            if not response:
                continue

            if response == "SUCCESS":
                return "SUCCESS"
            elif response == "FAILED" or response == "ERROR":
                return "FAILED"

    def initializeRobot(self):
        self.robotState = MachineState.RESETTING
        print(self.robotState)
        self.robotState = MachineState.IDLE
        print(self.robotState)

    # ====================================================
    #   Methods for Actions
    # ====================================================
    def jSpaceGRH(self,goal,finishGoal,feedbackPub,resultRequestCB):
        # Step 1: Extract data from goal message
        mode = goal["mode"]
        joint = goal["joint"]
        jogDistance = goal["jogDistance"]
        desiredPositions = goal["jointPositions"]
        currentPosition = self.jointState
        
        # Step 2: Parse goal (in degrees) for Arduino command
        arduinoCmd = self.parseMoveCommand(goal)

        # Step 3: Convert desired positions and travel distance to radians
        if joint != "Q1":
            jogDistance = np.deg2rad(jogDistance)

        for i in range(len(desiredPositions)):
            if self.robot.links[i].jointType == "revolute":
                desiredPositions[i] = np.deg2rad(desiredPositions[i])
        
        if mode == "REL":
            index = int(joint[1]) - 1 
            currentPosition[index] += jogDistance
            desiredPositions = currentPosition 

        # Step 4: Create Joint Trajectories
        self.createTrajectories(desiredPositions,"J")

        # Step 5: Send command to Arduino if in operation mode. 
        if self.operatingMode == OperatingMode.OPERATION:
            self.sendCommand(arduinoCmd)
        #print(arduinoCmd)
        
        # Step 5: Simulate Trajectory
        try:
            self.simulateTrajectory()
        except:
            return
        
        # Step 6: Return result after goal execution has finished
        finishGoal(resultRequestCB)
    
    def cSpaceGRH(self,goal,finishGoal,feedbackPub,resultRequstCB):
        self.robotState = MachineState.EXECUTING
        print(self.robotState)
        # Step 1: Extract data from goal message
        moveType = goal["moveType"]
        mode = goal["mode"]
        axis = goal["axis"]
        jogDistance = goal["jogDistance"]   # Units: X,Y,Z = mm , Phi = degrees
        desiredPose = goal["pose"]          # Units: X,Y,Z = mm , Phi = degrees
        

        # Step 2: Obtain current robot pose
        currentXYZ = self.poseState["POSITION"]     # Units: mm
        currentPHI = self.poseState["ANGLE"]        # Units: radians
        currentPose = [currentXYZ,currentPHI]       # [mm,mm,mm,radians]
        currentPose = np.concatenate([np.atleast_1d(item) for item in currentPose]).tolist()

        axisToIndex = {"X":0,"Y":1,"Z":2,"φ":3}

        if mode == "REL":
            if axis == "φ":
                currentPose[axisToIndex[axis]] += np.deg2rad(jogDistance)
                currentPose[axisToIndex[axis]] = np.rad2deg(currentPose[axisToIndex[axis]])
            else:
                currentPose[axisToIndex[axis]] += jogDistance
            desiredPose = currentPose
                
        # Step 3: Extract cartesian coordinates for robot IK
        x = desiredPose[0]
        y = desiredPose[1]
        z = desiredPose[2]
        
        phi = np.deg2rad(desiredPose[3])

        # Step 4: Perfrom inverse kinematics
        q1,q2,q3,q4 = self.robot.scaraIK(x,y,z,phi,self.desiredElbOri) # [mm,deg,deg,deg]

        desiredPositions = [q1,q2,q3,q4]

        # Re-construct goal for parsing
        goal = {
            "moveType": moveType,
            "mode": "ABS",
            "jointPositions": desiredPositions,
            }

        arduinoCmd = self.parseMoveCommand(goal)

        for i in range(len(desiredPositions)):
            if self.robot.links[i].jointType == "revolute":
                desiredPositions[i] = np.deg2rad(desiredPositions[i])

        # Step 5: Create Joint Trajectories
        self.createTrajectories(desiredPositions,"J")

        # Step 4: Send command to Arduino if in operation mode. 
        if self.operatingMode == OperatingMode.OPERATION:
           self.sendCommand(arduinoCmd)
        #print(arduinoCmd)
        
        # Step 5: Simulate Trajectory
        try:
            self.simulateTrajectory()
        except:
            return

        self.robotState = MachineState.COMPLETE
        self.robotState = MachineState.IDLE
        print(self.robotState)
        finishGoal(resultRequstCB)

    def gripObjGRH(self,goal,finishGoal,feedbackPub,resultRequestCB):
        # Step 1: Extract data from goal message
        gripType = goal["GRIP TYPE"]
        desiredGripDiameter = goal["GRIP DIAMETER"]

        # Step 2: Capture current Grip Diameter and Q4 Angle
        q4 = np.rad2deg(self.jointState[3])
        currentGripDiameter = self.gripDiameter

        # Step 3: Calculate relative distance the end effector must travel
        q4RelativeTravel = self.endEffector.determineSunGearTravel(currentGripDiameter,float(desiredGripDiameter))
        
        if self.operatingMode == OperatingMode.OPERATION:
            if gripType == "INTERNAL":
                # Send Command to Activate Solenoid
                self.sendCommand("solenoidon")
                

                # Send command to desired grip diameter
                self.sendCommand(f"REL,Q4:{round(q4RelativeTravel)}")


                # Send Command to De-activate Solenoiid
                self.sendCommand("solenoidoff")
                
  

                # Send command to help release solenoid pins from planet carrier slots.
                self.sendCommand("REL,Q4:-4")

                self.sendCommand("REL,Q4:4")
  
                # Reset Q4's Arduino Position
                self.gripDiameter = round(float(desiredGripDiameter))
                self.sendCommand(f"HOME,DIRECT,Q4:{str(q4)}") # Indexing in degrees


            elif gripType == "EXTERNAL":
                pass

        # Step 6: Return result after goal execution has finished
        finishGoal(resultRequestCB)

    def releaseObjGRH(self,goal,finishGoal,feedbackPub,resultRequestCB):
        # Step 1: Extract data from goal message
        gripType = goal["GRIP TYPE"]
        
        if gripType == "INTERNAL":
            desiredGripDiameter = self.endEffector.MIN_GRIP_DIAMETER + 5
        elif gripType == "EXTERNAL":
            desiredGripDiameter = self.endEffector.MAX_GRIP_DIAMETER

        # Step 2: Capture current Grip Diameter and Q4 Angle
        q4 = np.rad2deg(self.jointState[3])
        currentGripDiameter = self.gripDiameter
        print(f"Current Q4 Angle: {q4} | Current Grip Diameter{currentGripDiameter}")

        # Step 3: Calculate relative distance the end effector must travel
        q4RelativeTravel = self.endEffector.determineSunGearTravel(currentGripDiameter,float(desiredGripDiameter))
        
        if self.operatingMode == OperatingMode.OPERATION:
            if gripType == "INTERNAL":
                # Send Command to Activate Solenoid
                self.sendCommand("solenoidon")

                # Send command to desired grip diameter
                self.sendCommand(f"REL,Q4:{round(q4RelativeTravel)}")

                # Send Command to De-activate Solenoiid
                self.sendCommand("solenoidoff")

                # Send command to help release solenoid pins from planet carrier slots.
                self.sendCommand("REL,Q4:4")

                self.sendCommand("REL,Q4:-4")

                # Reset Q4's Arduino Position
                self.gripDiameter = round(float(desiredGripDiameter))
                self.sendCommand(f"HOME,DIRECT,Q4:{str(q4)}")
            
        # Step 6: Return result after goal execution has finished
        finishGoal(resultRequestCB)

    def parseMoveCommand(self,goalMsg):
        # Extract Move Type
        moveType = goalMsg["moveType"]
        
        # Joint Space Goal Parsing
        if moveType == "Joint Space":
            mode = goalMsg["mode"]
            joint = goalMsg["joint"]
            jogDistance = goalMsg["jogDistance"]
            jointPositions = goalMsg["jointPositions"].copy()
            currentPosition = self.jointState.copy()

            for i in range(len(jointPositions)):
                if self.robot.links[i].jointType == "prismatic" or self.robot.links[i].driveMechanism != "DIRECT":
                    position = jointPositions[i]
                    jointPositions[i] = self.robot.links[i].driveMechObj.findAngle(position)

            if joint == "Q1":
                endPosition = currentPosition[0] + jogDistance
                jogDistance = self.robot.links[0].driveMechObj.findAngleRelative(currentPosition[0],endPosition)

            if mode == "ABS":
                arduinoString = f"{mode},Q1:{jointPositions[0]},Q2:{jointPositions[1]},Q3:{jointPositions[2]},Q4:{jointPositions[3]}"
            elif mode == "REL":
                arduinoString = f"{mode},{joint}:{jogDistance}>"

        # Cartesian Space Goal Pasrsing
        elif moveType == "Cartesian Space":
            mode = goalMsg["mode"]
            jointPositions = goalMsg["jointPositions"].copy()

            for i in range(len(jointPositions)):
                if self.robot.links[i].jointType == "prismatic" or self.robot.links[i].driveMechanism != "DIRECT":
                    position = jointPositions[i]
                    jointPositions[i] = self.robot.links[i].driveMechObj.findAngle(position)

            arduinoString = f"{mode},Q1:{jointPositions[0]},Q2:{jointPositions[1]},Q3:{jointPositions[2]},Q4:{jointPositions[3]}"
        return arduinoString

    def toggleOpMode(self):
        if self.operatingMode == OperatingMode.OPERATION:
            self.operatingMode = OperatingMode.SIMULATION
            print("Controller now in SIMULATION MODE")
            return
        elif self.operatingMode == OperatingMode.SIMULATION:
            self.operatingMode = OperatingMode.OPERATION
            print("Controller now in OPERATION MODE")
    
    def toggleElbowOrient(self):
        if self.desiredElbOri == "elbowDown":
            self.desiredElbOri = "elbowUp"
        else:
            self.desiredElbOri = "elbowDown"
    
    def toggleMotorsOn(self):
        if self.operatingMode == OperatingMode.OPERATION:
            if self.motorsOn == False:
                self.sendCommand("motorsOn")
                self.motorsOn = True
            else:
                self.sendCommand("motorsOff")
                self.motorsOn = False
        

    def indexGRH(self,goal,finishGoal,feedbackPub,resultRequestCB):
        # Step 1: Extract data from goal message
        desiredMode = goal["Mode"]
        axesToIndexDict = goal["Axes to Index"]

        # Get Current Joint Positions
        currentJointPos = self.jointState
        
        # Step 2: Create a string of the axes to index using a generator expression and the str.join() method
        axesToIndexStr = ",".join(
            f"{axis}:{data['value']}"
            for axis,data in axesToIndexDict.items() if data["enabled"]
        )

        # Step 3: Define Lookup Table
        INDEX_MODE_LOOKUP = {mode.name: mode for mode in IndexMode}

        # Step 4: Set current index mode
        self.indexingMode = INDEX_MODE_LOOKUP.get(desiredMode)

        # Step 5: Format Arduino String and send command to Arduino
        arduinoCmd = f"HOME,{self.indexingMode.name},{axesToIndexStr}"

        if self.operatingMode == OperatingMode.OPERATION:
            self.sendCommand(arduinoCmd)
            print(arduinoCmd)
        else:
            print(f"Simulation Arduino Command: {arduinoCmd}")
        
        # Step 6: Update Controller States 
        for index,(_,value) in enumerate(axesToIndexDict.items()):
            if value["enabled"]:
                currentJointPos[index] = 0
                self.calibrationState[index] = True
        self.updateJointState(currentJointPos)
        
        # Step 7: Return result after goal execution has finished
        finishGoal(resultRequestCB)

    # ====================================================
    #   Methods trajectory generation and simulation
    # ====================================================
    def _initalizeTrajTargets(self):
        for link in self.robot.links:
            self.trajTargets["J-Space"][link.name] = {}
            self.trajTargets["J-Space"][link.name]["Target Position"] = 0
            self.trajTargets["J-Space"][link.name]["Target Speed"] = 0
        
        for DOF in ["X","Y","Z","","ψ(Yaw)","θ(Pitch)"," φ(Roll)"]:
            self.trajTargets["C-Space"][DOF] = 0

    def createTrajectories(self,desPosList,motionType:str = "J"):
        if motionType == "J":
            # Retrive current robot position
            currentPos = self.robot.jointPosition
            desPos = desPosList

            # Create Trajectories for each joint
            for i,_ in enumerate(desPos):
                trajObj = TrapezoidalTrajectory(currentPos[i],desPos[i],112.5,112.5,"1/3")
                self.trajectoryList[i] = trajObj
        elif motionType == "C":
            # Retrieve current robot Pose
            pass
    
    def _getJointPosRT(self,t):
        currPosList = []
        for trajectory in self.trajectoryList:
            currPosList.append(trajectory.positionRT(t))
        return currPosList

    def simulateTrajectory(self):
        # Find longest travel time
        travelTime = 0
        for traj in self.trajectoryList:
            if traj.tTotal > travelTime:
                travelTime = traj.tTotal
        print(f"{round(travelTime,2)} seconds")

        currentJointPosition = [0] * len(self.trajectoryList)

        # Start Timer
        baseTime = time.monotonic()
        elapsedTime = time.monotonic() - baseTime
        
        while elapsedTime <= travelTime:
            for i,traj in enumerate(self.trajectoryList):
                position = np.interp(elapsedTime,traj.timeVec, traj.trajectory[0])
                # Update robot joint position
                currentJointPosition[i] = position
                currPosFloatList = [float(x) for x in currentJointPosition] # Convert numpy float to float
            self.updateJointState(currPosFloatList)
            elapsedTime = time.monotonic() - baseTime # Update elapsed time
            time.sleep(0.1) 
        
        for i,traj in enumerate(self.trajectoryList):
            position = np.interp(elapsedTime,traj.timeVec, traj.trajectory[0])
            currentJointPosition[i] = position
            currPosFloatList = [float(x) for x in currentJointPosition] # Convert numpy float to float
        self.updateJointState(currPosFloatList)
         
    # ====================================================
    #   Methods for updating robot joint position and pose
    # ====================================================

    def publishCurrentState(self):
        # Retrieve latest information about joint positions and pose from controller
        jState = self.jointState
        cState = self.poseState
        calState = self.calibrationState
        gripD = self.gripDiameter
        # Publish state to topic
        self.publishers["jStatePub"].publishMsg(jState)
        self.publishers["cStatePub"].publishMsg(cState)
        self.publishers["calStatePub"].publishMsg(calState)
        self.publishers["gripDiameterPub"].publishMsg(gripD)
    
    def updateJointState(self,q):
        # Update information in robotArm object
        self.robot.updateJoints("ABS",q) # Performs FK as well

        # Update information on controller
        self.jointState = self.robot.jointPosition.copy()
        self.poseState = self.robot.pose.copy()

    # ====================================================
    #   Methods for Controller Looping
    # ====================================================
    
    def updateController(self):
        self.publishCurrentState()
        self.checkForGoalRequests()

    def checkForGoalRequests(self):
        for _,item in self.actionServers.items():
            item.handleGoalRequest()
    
    def practiceRoutine(self):
        try:
            i = 0
            while i < 5:
                arduinoCMD = "ABS,Q1:90,Q3:-90"
                self.sendCommand(arduinoCMD)
                time.sleep(10)
                arduinoCMD = "ABS,Q2:0,Q3:90"
                self.sendCommand(arduinoCMD)
                time.sleep(10)
                i += 1
            self.sendCommand("STOP")
        except:
            print("Unable to start Routine. Serial Communication not established.")


    # ====================================================
    #   Methods for Routines
    # ====================================================
    def createRoutineStep(self,moveType:str = None,moveMode:str = None,gripType:str = None,gripDiameter:float = None,jogDistance:float = None,joint:str = None, axis:str = None,jointPositions:list = None,pose:list = None,elbowOri:str = "elbowDown",posZOffset:float = 0, negZOffset:float = 0):
        goal = {
            "MOVE TYPE": moveType,
            "MOVE MODE": moveMode,
            "GRIP TYPE": gripType,
            "GRIP DIAMETER": gripDiameter,
            "JOG DISTANCE": jogDistance,
            "JOINT": joint,
            "JOINT POSITIONS": jointPositions,
            "AXIS": axis,
            "POSE": pose,
            "ELBOW ORIENTATION": elbowOri,
            "POSITIVE Z-OFFSET": posZOffset,
            "NEGATIVE Z-OFFSET": negZOffset,
            }
        return goal
    
    def defaultRoutine(self):
        routine = {}
        step1 = self.createRoutineStep(moveType = "Cartesian Space", moveMode = "ABS",pose = [191,191,0,0],elbowOri= "elbowUp")
        step2 = self.createRoutineStep(moveType = "Cartesian Space", moveMode = "ABS",pose = [191,191,25,0],elbowOri= "elbowUp")
        step3 = self.createRoutineStep(moveType = "Cartesian Space", moveMode = "ABS",pose = [191,0,25,60],elbowOri= "elbowUp")
        step4 = self.createRoutineStep(moveType = "Cartesian Space", moveMode = "ABS",pose = [191,0,3,60],elbowOri= "elbowUp")
        step6 = self.createRoutineStep(moveType = "Cartesian Space", moveMode = "ABS",pose = [191,0,25,60],elbowOri= "elbowUp")
        step7 = self.createRoutineStep(moveType = "Cartesian Space", moveMode = "ABS",pose = [191,191,25,0],elbowOri = "elbowUp")
        step8 = self.createRoutineStep(moveType = "Cartesian Space", moveMode = "ABS",pose = [191,191,0,0],elbowOri = "elbowUp")
        
        routine["1"] = step1
        routine["2"] = step2
        routine["3"] = step3
        routine["4"] = step4
        routine["6"] = step6
        routine["7"] = step7
        routine["8"] = step8
        return routine

    def runRoutine(self):
        if not self.routine:
            print("No routine present")
            return
        for routine,goal in self.routine.items():
            if goal["MOVE TYPE"] == "Cartesian Space":
                self.desiredElbOri = goal["ELBOW ORIENTATION"]
                cGoal = {
                    "moveType": goal["MOVE TYPE"],
                    "mode": "ABS",
                    "axis": "all",
                    "jogDistance": goal["JOG DISTANCE"],
                    "pose": goal["POSE"]
                    }
                self.actionClients["cCmdClient"].sendGoalRequest(cGoal)
                self.actionClients["cCmdClient"].waitForResult()

            elif goal["MOVE TYPE"] == "Joint Space":
                jGoal = {
                    "mode": goal["MOVE MODE"],
                    "joint": goal["JOINT"],
                    "jogDistance": goal["JOG DISTANCE"],
                    "jointPositions": goal["JOINT POSITIONS"],
                }
                print(jGoal)
            elif goal["MOVE TYPE"] == "Pick ":
                return