import serial
import time
import numpy as np

from CustomLibraries.PubSub import Publisher,Subscriber

class RobotController():
    # Contructor
    def __init__(self,robotObject,**kwargs):
        
        # Instantiate robot arm
        self.robot = robotObject

        # Add topics from **kwargs
        self.topics = {}
        for name,obj in kwargs.items():
            self.topics[name] = obj

        # Create dictionary for publishers and subscribers
        self.publishers = {}
        
        for key,item in self.topics.items():
            self.publishers[key + "Pub"] = Publisher(item)

        self.subscribers = {}

        # I/O for GUI (Controller logic)
        self.calibratedFlag = [False for _ in self.robot.links]
        self.serialConnect = False
        self.simulate = True
        
        # Controller states
        self.jointState = self.robot.jointPosition
        self.poseState = self.robot.pose

    # ======================================
    #   Methods for Serial Communication
    # ======================================

    def connectSerial(self):
        if "serialObj" not in self.__dict__:
            try:
                ser = serial.Serial(
                    port = "/dev/cu.usbmodem11101",
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

    # ====================================================
    #   Methods for updating robot joint position and pose
    # ====================================================
    def updateJointState(self,q):
        # Update information in robotArm object
        self.robot.updateJoints(q) # Performs FK as well
        
        # Update information on controller
        self.jointState = self.robot.jointPosition.copy()

        # Publish information to topic
        self.publishJointState()

    def updatePoseState(self,q):
        # Update information in robotArm object
        self.robot.updateJoints(q) # Performs FK as well
        
        # Update information on conroller
        self.poseState = self.robot.pose.copy()

        # Publish information to topic
        self.publishPoseState()       

    # ======================================
    #   Methods for Publishers and Subscriber
    # ======================================           
    
    def publish(self,topic,msg):
        pass


    def publishJointState(self):
        msg = {
            "Joint Positions": self.jointState,
            "timestamp": time.time()
        }
    
    def publishPoseState(self):
        msg = {
            "XYZ": self.poseState["POSITION"],
            "angle": self.poseState["ANGLE"],
            "timestamp": time.time()
        }
        print(msg)
    
    def jointCmdSubscriber(self):
        pass



"""
# ---- Example usage ----

# Create a topic for Cartesian poses
cPoseTopic = Topic("cartesian_pose")

# Create a publisher for that topic
cPosePub = Publisher(cPoseTopic)

# Create subscribers
cPoseSub_GUI= Subscriber("GUI")
cPoseSub_ARD = Subscriber("Arduino")

# Subscribe to topic
cPoseTopic.addSubscriber(cPoseSub_GUI)
cPoseTopic.addSubscriber(cPoseSub_ARD)

# Publish data 
for i in range (100):
    cPosePub.publishMsg(i)
    time.sleep(1)

"""