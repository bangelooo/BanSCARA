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

# ============= ROBOT LINK ======================
class RobotLink():
    # Constructor
    def __init__(self,alpha,a,d,theta,jointType,dhType,name:str,*args):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.jointType = jointType
        self.dhType = dhType
        self.name = name

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
        Keys: "FK" , "POSITION", "ANGLE"
        """
        q = self.jointPosition
        FK , _ = self.forwardKin(q)
        r11 = FK[0][0]
        r21 = FK[1][0]
        yaw = np.arctan2(r21,r11)
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
        Returns a tuple of the joint angles required to achieve the desired robot pose.
        
        :param x: x-coordinate
        :param y: y-coorindate
        :param z: z-coordinate
        :param phi: yaw-angle about the base frame
        :param elbowOrient: Elbow orientation
        '''
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
    L1,L2,L3,L4 = 1,191,191,0
    d4 = 1

    # Declare Links using Standard DH paramters
    link1 = RobotLink(0,0,L1,0,"prismatic","std", "q1")
    link2 = RobotLink(0,L2,0,0,"revolute","std", "q2")
    link3 = RobotLink(0,L3,0,0,"revolute","std", "q3")
    linkEE = RobotLink(-np.pi,L4,d4,0,"revolute","std", "q4")


    # Group robot links
    robotlinks = [link1,link2,link3,linkEE]

    # Create robot 
    scaraRobot = RobotArm(robotlinks)

    return scaraRobot

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

class RobotArmController():
    # Contructor
    def __init__(self,robotObject,topicsList,serviceList,actionsList):        # Instantiate robot arm
        self.robot = robotObject   

        # ======================================
        #   Publishers and Subscribers
        # ======================================
        # Set publishers and subscribers for robot controller
        pubs = ["jState","cState"]
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
        actionServers = ["jCmd","cCmd"]

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
        except:
            print("Action does not exist")
    
        # ======================================
        #   Controller Attributes
        # ======================================   
        # Logic
        self.operatingMode = OperatingMode.OPERATION
        self.serialConnect = False  
        self.calibrationState = [False for _ in self.robot.links]

        # Controller states
        self.robotState = MachineState.STOPPED
        self.jointState = self.robot.jointPosition
        self.poseState = self.robot.pose

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
            self.serialObj.open()
            self.serialConnect = True
            msg = "CONNECTED"
            return msg
    
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
        self.serialObj.write((cmd + '\n').encode('utf-8'))
        print("Sent",cmd)
        while self.serialObj.in_waiting:
            print(self.serialObj.readline().decode().strip())

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
        jogDistance = np.deg2rad(jogDistance)
        desiredPositions = [np.deg2rad(x) for x in desiredPositions] # Covert joint angles from degrees to radians
        
        if mode == "REL":
            index = int(joint[1]) - 1 
            currentPosition[index] += jogDistance
            desiredPositions = currentPosition 

        # Step 4: Create Joint Trajectories
        self.createTrajectories(desiredPositions,"J")

        # Step 4: Send command to Arduino if in operation mode. 
        if self.operatingMode == OperatingMode.OPERATION:
            self.sendCommand(arduinoCmd)
        print(arduinoCmd)
        
        # Step 5: Simulate Trajectory
        try:
            self.simulateTrajectory()
        except:
            return
        
        # Step 6: Return result after goal execution has finished
        finishGoal(resultRequestCB)

    def parseMoveCommand(self,goalMsg):
        moveType = goalMsg["moveType"]
        mode = goalMsg["mode"]
        joint = goalMsg["joint"]
        jogDistance = goalMsg["jogDistance"]
        jointPositions = goalMsg["jointPositions"]

        if moveType == "Joint Space":
            if mode == "ABS":
                arduinoString = f"{mode},Q1:{jointPositions[0]},Q2:{jointPositions[1]},Q3:{jointPositions[2]},Q4:{jointPositions[3]}"
            elif mode == "REL":
                arduinoString = f"{mode},{joint}:{jogDistance}>"

        elif moveType == "Cartesian Space":
            pass
        
        return arduinoString


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
                trajObj = TrapezoidalTrajectory(currentPos[i],desPos[i],1,100,"1/3")
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
        # Publish state to topic
        self.publishers["jStatePub"].publishMsg(jState)
        self.publishers["cStatePub"].publishMsg(cState)
    
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
            

    # def onJointCommand(self):
    #     # Read information from Joint Command
    #     if self.subscribers["jCmdSub"].msg == None:
    #         return
    #     else:
    #         msg = self.subscribers["jCmdSub"].msg
    #         id = msg["cmdID"]
    #         if self.lastCmdID == id:
    #             return
    #         else:
    #             # Prepare Command String
    #             if msg["joint"] == "all":
    #                 cmdStr = f"{msg["cmdID"]}<{msg["mode"]},Q1:{msg["jointPositions"][0]},Q2:{msg["jointPositions"][1]},Q3:{msg["jointPositions"][2]},Q4:{msg["jointPositions"][3]}>"
    #             else:
    #                 cmdStr = f"{msg["cmdID"]}<{msg["mode"]},{msg["joint"]}:{msg["jogDistance"]}>"
    #                 q = []
    #                 for i in msg["jointPositions"]:
    #                     i = float(i)
    #                     q.append(i)
      
                # print(cmdStr)
                # self.lastCmdID = id