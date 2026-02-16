import sys
import numpy as np
import time

from PyQt6.QtWidgets import (
    QMainWindow, QTabWidget, QWidget,
    QPushButton, QLineEdit,QTextEdit, QLabel,
    QVBoxLayout,QHBoxLayout,QGridLayout,
    QGroupBox
)

from PyQt6.QtCore import QRunnable, QThreadPool, QTimer, pyqtSlot

from .RobotCommunications import (
    Publisher,
    Subscriber,
    ActionClient
    )

#, ActionClient,ActionServer
class ControlWorker(QRunnable):
    def __init__(self,robotController):
        super().__init__()
        self.controller = robotController
    "Worker Thread"
    @pyqtSlot()
    def run(self):
        while True:
            self.controller.updateController()
            time.sleep(0.02) # 50 Hz

class RobotGUI(QMainWindow):
    def __init__(self,robotObject,robotController,topicList,serviceList,actionList):
        super().__init__()

        # Create Robot and Robot Controller Instance
        self.robot = robotObject
        self.controller = robotController
        self.controlLoop = ControlWorker(self.controller)
        self.stateTimer = QTimer()
        self.stateTimer.setInterval(1000) # ms
        self.stateTimer.timeout.connect(self.updateStates)
        self.stateTimer.start()

        # ======================================
        #   Publishers and Subscribers
        # ======================================
        # Set publishers and subscribers for robot controller
        subs = ["jState","cState"]
        pubs = ["jCmd","cCmd"]

       # Create toics dictionary
        self.topics = {}

        for topicObj in topicList:
            self.topics[topicObj.name] = topicObj

        # Create dictionary for publishers and subscribers
        self.publishers = {}
        for key,item in self.topics.items():
            if key in pubs:
                self.publishers[key + "Pub"] = Publisher(item)

        self.subscribers = {}
        for key,item in self.topics.items():
            if key in subs:
                sub = self.subscribers[key + "Sub"] = Subscriber(key)
                # Subscribe to the topic
                self.topics[key].addSubscriber(sub)
    
        # ======================================
        #   Action Clients and Action Servers
        # ======================================
        # Set action clients for robot controller
        actionClients = ["jCmd","cCmd"]

        # Create dictionary for actions
        self.actions = {}

        for actionObj in actionList:
            self.actions[actionObj.name] = actionObj
        
        # Create dictionary for action clients
        self.actionClients = {}

        for key,item in self.actions.items():
            if key in actionClients:
                self.actionClients[key + "Client"] = ActionClient(item)
    
        # Create logger Count
        self.logCount = 0
        self.cmdID = 0

        # Establish Dictionary Keys
        GROUP = ["Robot Control", 
                 "Cartesian Control", 
                 "Joint Space Control",
                 "Calibration Settings", 
                 "Axes Calibrated",
                 "Cartesian Control Settings",
                 "Joint Space Control Settings"]
    
        # Adjust window parameters
        self.setWindowTitle("SCARA Robot Controls")
        self.resize(1000,900)

        # Dictionaries for buttons
        self.buttonDict = {}
        self.textBoxDict = {}

        # Create tab structure for main window
        tabs = QTabWidget()
        tabs.addTab(self.mainWindow(), "MAIN")

        self.setCentralWidget(tabs)

        # Check if button clicked

        for _, buttonGroupDict in self.buttonDict["Joint Space Control"].items():
            buttonGroupDict["button"].clicked.connect(self.sendJCmdGoalRequest)

        for _, buttonGroupDict in self.buttonDict["Cartesian Control"].items():
            buttonGroupDict["button"].clicked.connect(self.publishcJogCommand)

        self.buttonDict["Robot Control"]["CONNECT"]["button"].clicked.connect(self.controller.connectSerial)
        self.buttonDict["Robot Control"]["DISCONNECT"]["button"].clicked.connect(self.disconnect)
        self.buttonDict["String Command"]["output"]["button"].clicked.connect(self.clearLog)

    # ======================================
    #   Multi-Threading
    # ======================================
        # Create ThreadPool and Timer
        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.start()

        self.threadpool = QThreadPool()

        self.threadpool.start(self.controlLoop)
# ==================================================================  
#     METHODS TO CREATE PANELS IN MAIN TAB 
# ==================================================================     
    def mainWindow(self):
        container = QWidget()
        layout = QHBoxLayout(container)

        # Create panels
        cp = self.controlPanel()
        rsp = self.robotStatePanel()
        slp = self.stringLogPanel()

        # Add widgets to layout
        layout.addWidget(cp)
        layout.addWidget(rsp)
        layout.addWidget(slp)

        return container
    
    def controlPanel(self):
        container = QWidget()
        cpLayout = QVBoxLayout(container)

        # Create widget groups
        rcGroup,rcDict = self.createRobotControlGroup()
        cCtrlGroup, cCtrlDict = self.createCCtrlGroup()
        jJogGroup,jJogDict = self.createJCtrlGroup()


        # Add buttons to desired dictionary
        self.buttonDict["Robot Control"] = rcDict
        self.buttonDict["Cartesian Control"] = cCtrlDict
        self.buttonDict["Joint Space Control"] = jJogDict

        # Add widget to control panel layout
        cpLayout.addWidget(rcGroup)
        cpLayout.addWidget(cCtrlGroup)
        cpLayout.addWidget(jJogGroup)

        return container

    def robotStatePanel(self):
        container = QWidget()
        rsLayout = QVBoxLayout(container)

        # Create Widget Groups
        cStateGroup,cStateDict = self.createcStateGroup()
        jStateGroup,jStateDict = self.createjStateGroup()

        # Add desired dictionaries
        self.textBoxDict["Cartesian Control Settings"] = cStateDict
        self.textBoxDict["Joint Space Control Settings"] = jStateDict

        # Add lineEdit to desired dictionary
        rsLayout.addWidget(cStateGroup)
        rsLayout.addWidget(jStateGroup)

        return container

    def stringLogPanel(self):
        container = QWidget()
        scLayout = QVBoxLayout(container)

        # Create Widget Group
        scGroup,strButtDict,strTxtDict = self.createStringCmdGroup()

        # Add desired Dictionaries
        self.textBoxDict["String Command"] = strTxtDict
        self.buttonDict["String Command"] = strButtDict

        scLayout.addWidget(scGroup)

        return container

# ==================================================================  
#     METHODS TO CREATE GROUPINGS FOR PANELS
# ==================================================================  
    def createRobotControlGroup(self):
        container = QGroupBox("ROBOT CONTROL")
        layout = QVBoxLayout(container)

        # Button Dictionary
        rcDict = {}

        for name in ["TEST","CONNECT","DISCONNECT"]:
            btn = QPushButton(name)
            rcDict[name] = {}
            rcDict[name]["button"] = btn
            layout.addWidget(btn)
        
        container.setLayout(layout)

        return container, rcDict

    def createCCtrlGroup(self):
        container = QGroupBox("CARTESIAN CONTROL")
        layout = QGridLayout(container)
 
        # Button Dictionary
        cCtrlDict = {}

        # Go to Pose Push Button
        goToPose = QPushButton("GO TO POSE") # Create button
        layout.addWidget(goToPose,0,0,1,2) # Add to layout
        cCtrlDict["GoToPose"]= {}
        cCtrlDict["GoToPose"]["button"] = goToPose
        cCtrlDict["GoToPose"]["DOF"] = "all"

        for index, DOF in enumerate(["X","Y","Z","φ"],start = 1):
            for sign in ["-","+"]:
                btn = QPushButton(DOF + sign)
                cCtrlDict[DOF + sign] = {}
                cCtrlDict[DOF + sign]["button"] = btn
                cCtrlDict[DOF + sign]["DOF"] = DOF
                if sign == "-":
                    cCtrlDict[DOF + sign]["direction"] = -1
                elif sign == "+":
                    cCtrlDict[DOF + sign]["direction"] = 1
            
            # Add button to QGridLayout
            layout.addWidget(cCtrlDict[DOF + "-"]["button"],index,0)
            layout.addWidget(cCtrlDict[DOF + "+"]["button"],index,1)

        # Add layout to QGroup Widget
        container.setLayout(layout)

        return container,cCtrlDict

    def createJCtrlGroup(self):
        container = QGroupBox("JOINT SPACE CONTROL")
        layout = QGridLayout(container)

        # Dictionary for Joint Space Jog Buttons
        jCtrlDict = {}

        # Go to Pose Push Button
        goToJntPos = QPushButton("GO TO JOINT POSITIONS") # Create button
        layout.addWidget(goToJntPos,0,0,1,2) # Add to layout
        jCtrlDict["GoToJntPos"]= {}
        jCtrlDict["GoToJntPos"]["button"] = goToJntPos
        jCtrlDict["GoToJntPos"]["joint"] = "all"

        for index,joint in enumerate(["Q1","Q2","Q3","Q4"],start = 1):
            for sign in ["-","+"]:
                btn = QPushButton(joint + sign)
                jCtrlDict[joint + sign] = {}
                jCtrlDict[joint + sign]["button"] = btn
                jCtrlDict[joint + sign]["joint"] = joint
                if sign == "-":
                    jCtrlDict[joint + sign]["direction"] = -1
                elif sign == "+":
                    jCtrlDict[joint + sign]["direction"] = 1
            
            # Add button to QGridLayout
            layout.addWidget(jCtrlDict[joint + "-"]["button"],index,0)
            layout.addWidget(jCtrlDict[joint + "+"]["button"],index,1)
        
        # Add layout to QGroup Widget
        container.setLayout(layout)

        return container,jCtrlDict

    def createcStateGroup(self):
        container = QGroupBox("CARTESIAN CONTROL SETTINGS")
        layout = QVBoxLayout(container)

        titleRow = QHBoxLayout()
        for DOF in ["","X","Y","Z","φ"]:
            lbl = QLabel(DOF)
            titleRow.addWidget(lbl)
            
        layout.addLayout(titleRow)

        # Create Dictionary for each Axis
        dictionary = {}
        for DOF in ["X","Y","Z","φ"]:
            dictionary[DOF] = {}
        
        # Create Widgets
        for rowIndex,label in enumerate(["Desired Pose", "Current Pose", "Jog Increment"]):
            row = QHBoxLayout()
            lbl = QLabel(label)
            row.addWidget(lbl)

            for DOF in ["X","Y","Z","φ"]:
                txtFld = QLineEdit("0.0")
                if rowIndex == 1:
                    txtFld.setReadOnly(True)
                # Add to new label to dictionary
                dictionary[DOF][label] = txtFld
                row.addWidget(txtFld)

            layout.addLayout(row)
        # Add Layout ot QGroupBox
        container.setLayout(layout)

        return container,dictionary

    def createjStateGroup(self):
        container = QGroupBox("JOINT SPACE CONTROL SETTINGS")
        layout = QVBoxLayout(container)

        titleRow = QHBoxLayout()
        for DOF in ["","Q1","Q2","Q3","Q4"]:
            lbl = QLabel(DOF)
            titleRow.addWidget(lbl)
            
        layout.addLayout(titleRow,0)

        # Create Dictionary for each Axis
        dictionary = {}
        for DOF in ["Q1","Q2","Q3","Q4"]:
            dictionary[DOF] = {}
        
        # Create Widgets
        for rowIndex,label in enumerate(["Desired Position", "Current Position", "Jog Increment"]):
            row = QHBoxLayout()
            lbl = QLabel(label)
            row.addWidget(lbl)

            for DOF in ["Q1","Q2","Q3","Q4"]:
                txtFld = QLineEdit("0.0")
                if rowIndex == 1:
                    txtFld.setReadOnly(True)
                # Add to new label to dictionary
                dictionary[DOF][label] = txtFld
                row.addWidget(txtFld)

            layout.addLayout(row)
        # Add Layout ot QGroupBox
        container.setLayout(layout)

        return container,dictionary

    def createStringCmdGroup(self):
        container = QGroupBox("STRING COMMANDS")
        layout = QVBoxLayout(container)
        
        # Create Widgets
        row = QHBoxLayout()

        inputFld = QLineEdit()
        sndStrBtn = QPushButton("SEND")
        row.addWidget(inputFld)
        row.addWidget(sndStrBtn)

        outputFld = QTextEdit()
        outputFld.setReadOnly(True)

        clrOutputFld = QPushButton("CLEAR OUTPUT FIELD")
       
       
        # # Add to dictionary 
        buttDict = {}
        txtDict = {}
        buttDict["input"] = {
            "button": sndStrBtn
            }
        buttDict["output"] = {
            "button": clrOutputFld
        }
        txtDict["input"] = {"field":inputFld}
        txtDict["output"] = {"field":outputFld}

        layout.addLayout(row)
        layout.addWidget(clrOutputFld)
        layout.addWidget(outputFld)

        # Add Layout to QGroupbox
        container.setLayout(layout)

        return container,buttDict,txtDict

# ==================================================================  
#     METHODS TO BUTTON BEHAVIORS
# ==================================================================  

    def connect(self):
        if not self.controller.serialConnect:
            msg = self.controller.connectSerial()
            self.updateLog("SUCCESS: " + msg)
        else:
            self.updateLog("Already connected")
        
    def disconnect(self):
        if self.controller.serialConnect:
            msg = self.controller.disconnectSerial()
            self.updateLog(msg)
        else:
            self.updateLog("Already disconnected")
    
    def sendJCmdGoalRequest(self):
        sender = self.sender() # Determine which button was preseed

        # Initialize jog parameters
        moveType = "Joint Space"
        joint = ""
        jogDistance = 0
        direction = 1
        jointPositions = []
        mode = ""

        # Search dictionary where button is located and set jog paramters
        for _, buttonGroupDict in self.buttonDict["Joint Space Control"].items():
            if buttonGroupDict["button"] == sender:
                joint = buttonGroupDict["joint"]
                if "direction" in buttonGroupDict:
                    direction= buttonGroupDict["direction"]
                mode = "REL"
                if joint == "all":
                    mode = "ABS"
                break
        
        # Search textField dictionary to set jogDistance
        for key, textFieldDict in self.textBoxDict["Joint Space Control Settings"].items():
            if key == joint:
                jogDistance = float(textFieldDict["Jog Increment"].text())
                break

        # Get Absolute Joint Position
        for j in ["Q1","Q2","Q3","Q4"]:
            qDes= self.textBoxDict["Joint Space Control Settings"][j]["Desired Position"].text()
            jointPositions.append(qDes)

        # Convert goal attributes to the correct data type
        jointPositions = [float(x) for x in jointPositions]

        goal = {
            "moveType": moveType,
            "mode": mode,
            "joint": joint,
            "jogDistance": jogDistance * direction,
            "jointPositions": jointPositions,
        }
        print(f"Type moveType:{moveType} | mode: {mode} | joint: {joint} | Jog Distance: {jogDistance * direction} | jointPositions: {jointPositions}")
        print(f"Type moveType:{type(moveType)} | mode: {type(mode)} | joint: {type(joint)} | Jog Distance: {type(jogDistance * direction)} | jointPositions: {type(jointPositions[0])}")

        self.actionClients["jCmdClient"].sendGoalRequest(goal)   
        
    def publishcJogCommand(self):
        self.updateRobotState()

        self.cmdID += 1

        # Identify pushed button
        sender = self.sender()

        # Intialize required variables
        axis = ""
        jogDistance = 0
        direction = 1
        pose = []

        # Search button dictionary where button object is located and set jog parameters
        for _,buttonGroupDict in self.buttonDict["Cartesian Control"].items():
            if buttonGroupDict["button"] == sender:
                axis = buttonGroupDict["DOF"]
                if axis == "all":
                    direction = 0
                elif "direction" in buttonGroupDict:
                    direction = buttonGroupDict["direction"]
                break
        
        # Search textBlock dictionary and set jog distance
        for key, textBoxDict in self.textBoxDict["Cartesian Control Settings"].items():
            if key == axis:
                jogDistance = float(textBoxDict["Jog Increment"].text())
                break

        # Get lastet desired pose
        for DOF in ["X","Y","Z","φ"]:
            desPose = self.textBoxDict["Cartesian Control Settings"][DOF]["Desired Pose"].text()
            pose.append(desPose)
        
        pose = [float(item) for item in pose]
        
        # Publish Cartesian Jog Message
        pubMsg = {
            "cmdID":self.cmdID,
            "axis": axis,
            "jogDistance": jogDistance * direction,
            "pose": pose
        }
        self.publishers["cCmdPub"].publishMsg(pubMsg)

# ==================================================================  
#     METHODS TO TEXT FIELD BEHAVIORS
# ==================================================================  
    def updateStates(self):
        # Read jState and cState subscribers
        jState = self.subscribers["jStateSub"].msg
        cState = self.subscribers["cStateSub"].msg
        
        # Update joint state fields in GUI
        for i, (_,subDict) in enumerate(self.textBoxDict["Joint Space Control Settings"].items()):
            subDict["Current Position"].setText(str(np.rad2deg(jState[i])))
        
        # Update pose state fields in GUI
        for i, (_,subDict) in enumerate(self.textBoxDict["Cartesian Control Settings"].items()):
            if i < 3:
                pos = str(round(cState["POSITION"][i],2))
                subDict["Current Pose"].setText(pos)
            else:
                angle = round(np.rad2deg(cState["ANGLE"]),2)
                angle = str(angle)
                subDict["Current Pose"].setText(angle)
        
    def updateRobotState(self):
        # Update Current Cartesian Cartesian Position 
        pose = self.robot.pose["POSITION"]
        angle  = self.robot.pose["ANGLE"]
        for index,DOF in enumerate(["X","Y","Z"]):
            self.textBoxDict["Cartesian Control Settings"][DOF]["Current Pose"].setText(str(pose[index]))

        self.textBoxDict["Cartesian Control Settings"]["φ"]["Current Pose"].setText(str(angle))
        
        # Update Joint Space Angles
        q = self.robot.jointPosition
        q = np.rad2deg(q)

        for index,j in enumerate(["Q1","Q2","Q3","Q4"]):
            self.textBoxDict["Joint Space Control Settings"][j]["Current Position"].setText(str(q[index]))

    def updateLog(self,logStr):
        self.logCount += 1
        log = self.textBoxDict["String Command"]["output"]["field"]
        log.append(logStr + str(self.logCount))

    def clearLog(self):
        self.textBoxDict["String Command"]["output"]["field"].clear()
        