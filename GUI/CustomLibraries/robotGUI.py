import sys
import numpy as np
import time

from PyQt6.QtWidgets import (
    QMainWindow, QTabWidget, QWidget,
    QPushButton, QLineEdit,QTextEdit, QLabel,
    QVBoxLayout,QHBoxLayout,QGridLayout,
    QCheckBox,
    QGroupBox,
    QButtonGroup,
    QSlider,
)

from PyQt6.QtCore import QRunnable, QThreadPool, QTimer, pyqtSlot,Qt

from .RobotCommunications import (
    Publisher,
    Subscriber,
    ActionClient
    )


# Widget Helper Classes
def createToggleSwitchGroup(labelName):
    toggleRow = QHBoxLayout()
    toggleSwitch = ToggleSwitch()
    lbl = QLabel(labelName)
    
    toggleRow.addWidget(lbl)
    toggleRow.addWidget(toggleSwitch)

    return toggleRow,toggleSwitch

class ToggleSwitch(QCheckBox):
    def __init__(self):
        super().__init__()
        self.setFixedSize(50,28)
        self.setStyleSheet("""
        QCheckBox {
            background-color: #ccc;
            border-radius: 14px;
        }
        QCheckBox::indicator {
            width: 26px;
            height: 26px;
            border-radius: 13px;
            background: white;
            margin: 1px;
        }
        QCheckBox::indicator:checked {
            margin-left: 22px;
        }
        QCheckBox:checked {
            background-color: #4CAF50;
        }
        """)

def createStatusLEDGroup(labelName):
    statusLEDRow = QHBoxLayout()
    statusLED = StatusLED()
    lbl = QLabel(labelName)

    statusLEDRow.addWidget(lbl)
    statusLEDRow.addWidget(statusLED)

    statusLEDRow.setAlignment(Qt.AlignmentFlag.AlignLeft)
    statusLEDRow.setSpacing(6)
    #statusLEDRow.addStretch()

    return statusLEDRow, statusLED

class StatusLED(QLabel):
    def __init__(self,diameter = 18):
        super().__init__()
        self.diameter = diameter
        self.setFixedSize(diameter, diameter)
        self.setState(False)


    def setState(self, state:bool):
        color = "green" if state else "red"
        self.setStyleSheet(f"""
            background-color: {color};
            border-radius: {self.diameter // 2}px;
            border: 1px solid black;
        """)

class ControlWorker(QRunnable):
    def __init__(self,robotController):
        super().__init__()
        self.controller = robotController
    "Worker Thread"
    @pyqtSlot()
    def run(self):
        while True:
            self.controller.updateController()
            #time.sleep(0.02) # 50 Hz

class ValueSliderWidget(QWidget):
    def __init__(self, minValue = 0, maxValue = 100, initial = 50,valueName = "Default",grouping = "Default", dictionary = None):
        super().__init__()

        self.minValue = minValue
        self.maxValue = maxValue
        self.valueName = valueName

        layout = QHBoxLayout()

        self.label = QLabel(valueName)
        layout.addWidget(self.label)

        # Line Edit for Manual Input
        self.lineEdit = QLineEdit(str(initial))
        layout.addWidget(self.lineEdit)

        # Add Line Edit Widget to dictionary
        dictionary[grouping] = {valueName:self.lineEdit}

        # Slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(minValue)
        self.slider.setMaximum(maxValue)
        layout.addWidget(self.slider)

        self.setLayout(layout)

        # =========== CONNECTIONS ====================
        self.slider.valueChanged.connect(self.updateFromSlider)
        self.lineEdit.editingFinished.connect(self.updateFromText)

    def updateFromSlider(self,value):
        """ Update text when slider moves"""
        self.lineEdit.setText(str(value))

    def updateFromText(self):
        """Update slider when text changes"""
        try:
            value = int(self.lineEdit.text())

            # Clamp Value
            value = max(self.minValue, min(self.maxValue,value))

            self.slider.setValue(value)
            self.lineEdit.setText(str(value)) # Ensure a valid dipslay
        except ValueError:
            # Reset to Slider value if invalid input
            self.lineEdit.setText(str(self.slider.value()))

    def getValue(self):
        return self.slider.value()

    def set_value(self, value):
        value = max(self.minValue, min(self.maxValue, value))
        self.slider.setValue(value)

# Robot GUI Main Window
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

       # Create topics dictionary
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
        actionClients = ["jCmd","cCmd","calCmd","gripCmd","releaseCmd"]

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

        for key, buttonGroupDict in self.buttonDict["Cartesian Control"].items():
            if key != "ELBOW ORIENT":
                buttonGroupDict["button"].clicked.connect(self.sendCCmdGoalRequest)

        self.buttonDict["Cartesian Control"]["ELBOW ORIENT"]["button"].clicked.connect(self.onToggleElbowOrient)

        self.buttonDict["Robot Control"]["CONNECT"]["button"].clicked.connect(self.controller.connectSerial)
        self.buttonDict["Robot Control"]["DISCONNECT"]["button"].clicked.connect(self.disconnect)
        self.buttonDict["String Command"]["output"]["button"].clicked.connect(self.clearLog)

        self.buttonDict["Robot Control"]["OPERATION MODE TOGGLE"]["button"].toggled.connect(self.controller.toggleOpMode)
        self.buttonDict["Robot Control"]["START ROUTINE"]["button"].clicked.connect(self.controller.practiceRoutine)
        self.buttonDict["Robot Control"]["MOTOR ON TOGGLE"]["button"].toggled.connect(self.controller.toggleMotorsOn)
        
        self.buttonDict["Robot Control"]["CALIBRATE"]["button"].clicked.connect(self.sendIndexGoalRequest)
        self.buttonDict["Robot Control"]["GRIP OBJECT"]["button"].clicked.connect(self.sendGripObjectGoalRequest)
        self.buttonDict["Robot Control"]["RELEASE OBJECT"]["button"].clicked.connect(self.sendReleaseObjectGoalRequest)
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
        cSettingGroup,cSettingDict = self.createCalibrationSettingGroup()
        eeSettingGroup,eeSettingDict = self.createEESettingGroup()
        cStateGroup,cStateDict = self.createcStateGroup()
        jStateGroup,jStateDict = self.createjStateGroup()

        # Add desired dictionaries
        self.buttonDict["Calibration Settings"] = cSettingDict
        self.buttonDict["End Effector Settings"] = eeSettingDict
        self.textBoxDict["Cartesian Control Settings"] = cStateDict
        self.textBoxDict["Joint Space Control Settings"] = jStateDict
        

        # Add lineEdit to desired dictionary
        rsLayout.addWidget(cSettingGroup)
        rsLayout.addWidget(eeSettingGroup)
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
    def createEESettingGroup(self):
        container = QGroupBox("END EFFECTOR SETTINGS")
        layout = QVBoxLayout(container)

        # Button Dictionary
        eeDict = {}

        # Row: Grip Orientation
        gripTypeRow = QHBoxLayout()
        gripRowTypeLbl = QLabel ("GRIP ORIENTATION")
        gripTypeRow.addWidget(gripRowTypeLbl)

        gripTypeBtnGroup = QButtonGroup(container)
        gripTypeBtnGroup.setExclusive(True)

        for name in ["INTERNAL", "EXTERNAL"]:
            btn = QPushButton(name)
            btn.setCheckable(True)
            eeDict[name] = {}
            eeDict[name]["button"] = btn
            gripTypeRow.addWidget(btn)
            gripTypeBtnGroup.addButton(btn)

        valueRow = ValueSliderWidget(15,110,15,"GRIP DIAMETER (mm)","End Effector Control",self.textBoxDict)

        layout.addLayout(gripTypeRow)
        layout.addWidget(valueRow)

        return container,eeDict

    def createCalibrationSettingGroup(self):
        container = QGroupBox("CALIBRATION SETTINGS")
        layout = QVBoxLayout(container)
        # Button Dictionary
        csDict = {"AXIS":{},"TYPE":{}}
        
        # Row 1: Axes to Calibrate
        calAxisRow = QHBoxLayout()
        row1Lbl = QLabel("CALIBRATE AXIS")
        calAxisRow.addWidget(row1Lbl)

        for name in ["Q1","Q2","Q3","Q4"]:
            btn = QPushButton(name)
            btn.setCheckable(True)
            csDict["AXIS"][name] = {}
            csDict["AXIS"][name]["button"] = btn
            calAxisRow.addWidget(btn)

        # Row 2: Calibration Type 
        calTypeRow = QHBoxLayout()
        row2Lbl = QLabel("CALIBRATION TYPE")
        calTypeRow.addWidget(row2Lbl)

        calTypeBtnGroup = QButtonGroup(container)
        calTypeBtnGroup.setExclusive(True)

        for name in ["DIRECT","TO FLAG", "HARD STOP"]:
            btn = QPushButton(name)
            btn.setCheckable(True)
            csDict["TYPE"][name] = {}
            csDict["TYPE"][name]["button"] = btn
            calTypeRow.addWidget(btn)
            calTypeBtnGroup.addButton(btn)


        # Row 3: Axes Calibrated
        calStatusRow = QHBoxLayout()
        calStatusLbl = QLabel("STATUS")
        calStatusRow.addWidget(calStatusLbl)

        for name in ["Q1", "Q2", "Q3", "Q4"]:
            statusLEDGroup,statusLED = createStatusLEDGroup(name)
            calStatusRow.addLayout(statusLEDGroup)

        layout.addLayout(calStatusRow)
        layout.addLayout(calAxisRow)
        layout.addLayout(calTypeRow)

        container.setLayout(layout)

        return container,csDict

    def createRobotControlGroup(self):
        container = QGroupBox("ROBOT CONTROL")
        layout = QVBoxLayout(container)

        # Button Dictionary
        rcDict = {}

        # Toggle Switch for Simulation
        opModeRow,opModeToggle = createToggleSwitchGroup("OPERATION MODE")

        layout.addLayout(opModeRow)
        rcDict["OPERATION MODE TOGGLE"] = {}
        rcDict["OPERATION MODE TOGGLE"]["button"] = opModeToggle

        # Toggle Switch for Motors On
        mtrOnRow,mtrOnToggle = createToggleSwitchGroup("MOTOR ON")

        layout.addLayout(mtrOnRow)
        rcDict["MOTOR ON TOGGLE"] = {}
        rcDict["MOTOR ON TOGGLE"]["button"] = mtrOnToggle

        for name in ["CONNECT","DISCONNECT","CALIBRATE","GRIP OBJECT","RELEASE OBJECT","START ROUTINE","STOP ROUTINE"]:
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


        # Elbow Orientation Push Button
        elbowOrient = QPushButton("ELBOW DOWN")
        elbowOrient.setCheckable(True)
        elbowOrient.setChecked(False)
        cCtrlDict["ELBOW ORIENT"] = {}
        cCtrlDict["ELBOW ORIENT"]["button"] = elbowOrient

        layout.addWidget(elbowOrient,1,0,1,2)

        for index, DOF in enumerate(["X","Y","Z","φ"],start = 2):
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
        sender = self.sender() # Determine which button was pressed

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
    
    def sendCCmdGoalRequest(self):
        sender = self.sender() # Determine which button was pressed

        # Initialize cartesian jog paramters
        moveType = "Cartesian Space"
        axis = ""
        elbowOrient = ""
        jogDistance = 0
        direction = 1
        pose = []
        mode = ""

        # Search dictionary where button is loacted and set jog parameters
        for _,buttonGroupDict in self.buttonDict["Cartesian Control"].items():
            if buttonGroupDict["button"] == sender:
                axis = buttonGroupDict["DOF"]
                if axis == "all":
                    direction = 0
                    mode = "ABS"
                elif "direction" in buttonGroupDict:
                    direction = buttonGroupDict["direction"]
                    mode = "REL"
                break
        
        # Search textBlock dictionary and set jog distance
        for key, textBoxDict in self.textBoxDict["Cartesian Control Settings"].items():
            if key == axis:
                jogDistance = float(textBoxDict["Jog Increment"].text())
                break
        
        # Get desired pose information
        for DOF in ["X","Y","Z","φ"]:
            desPose = self.textBoxDict["Cartesian Control Settings"][DOF]["Desired Pose"].text()
            pose.append(desPose)
        
        # Convert data type
        pose = [float(item) for item in pose]

        # Create Goal
        goal = {
            "moveType":moveType,
            "mode": mode,
            "axis": axis,
            "jogDistance": jogDistance * direction,
            "pose": pose
        }
        print(goal)

        self.actionClients["cCmdClient"].sendGoalRequest(goal)   

    def onToggleElbowOrient(self):
        sender = self.sender()      # Determine which button was pressed
        toggled = sender.isChecked()
        if toggled:
            sender.setText("ELBOW UP")
            self.controller.toggleElbowOrient()
        else:
            sender.setText("ELBOW DOWN")
            self.controller.toggleElbowOrient()

    def sendIndexGoalRequest(self):
        sender = self.sender()  # Determine which button was pressed
        
        # Initialize Calibration Parameters
        calType = ""
        axesToIndex = {
            "Q1": {"enabled": False, "value": 0},
            "Q2": {"enabled": False, "value": 0},
            "Q3": {"enabled": False, "value": 0},
            "Q4": {"enabled": False, "value": 0},
        }
        
        # Check which
        for calTypeKey,calTypeData in self.buttonDict["Calibration Settings"]["TYPE"].items():
            button = calTypeData["button"]
            if button.isChecked():
                calType = calTypeKey
                # Remove all white spaces
                
                break
        calType =calType.replace(" ","")
        # Check Axes to Calibrate
        for axis,axisData in self.buttonDict["Calibration Settings"]["AXIS"].items():
            button = axisData["button"]
            if button.isChecked():
                axesToIndex[axis]["enabled"] = True
        
        # Construct Goal
        goal = {"Mode":calType,
                "Axes to Index":axesToIndex}
        
        # Send Goal Request
        print(f"Client Request: {goal}")

        self.actionClients["calCmdClient"].sendGoalRequest(goal)

    def sendGripObjectGoalRequest(self):
        sender = self.sender()  # Determine which button was pressed

        # Step 1: Initialize Grip Parameters
        gripType = ""
        gripDiameter = None

        # Step 2: Check which grip type is selected (Internal or External)
        for eeGripTypeKey,eeGripTypeData in self.buttonDict["End Effector Settings"].items():
            button = eeGripTypeData["button"]
            if button.isChecked():
                gripType = eeGripTypeKey
                break
        
        # Step 3: Populate the desired grip diameter
        gripDiameter = self.textBoxDict["End Effector Control"]["GRIP DIAMETER (mm)"].text()

        # Step 4: Construct Goal
        goal = {
            "GRIP TYPE": gripType,
            "GRIP DIAMETER": gripDiameter
            }
        
        # Step 5: Send Goal Request
        print(f"Client Request: {goal}")
        self.actionClients["gripCmdClient"].sendGoalRequest(goal)

    
    def sendReleaseObjectGoalRequest(self):
        sender = self.sender()  # Determine which button was pressed

        # Step 1: Initialize Grip Parameters
        gripType = ""

        # Step 2: Check which grip type is selected (Internal or External)
        for eeGripTypeKey,eeGripTypeData in self.buttonDict["End Effector Settings"].items():
            button = eeGripTypeData["button"]
            if button.isChecked():
                gripType = eeGripTypeKey
                break

        # Step 3: Construct Goal
        goal = {
            "GRIP TYPE": gripType,
            }
        
        # Step 5: Send Goal Request
        print(f"Client Request: {goal}")
        self.actionClients["releaseCmdClient"].sendGoalRequest(goal)

# ==================================================================  
#     METHODS TO TEXT FIELD BEHAVIORS
# ==================================================================  
    def updateStates(self):
        # Read jState and cState subscribers
        jState = self.subscribers["jStateSub"].msg
        cState = self.subscribers["cStateSub"].msg
        
        # Update joint state fields in GUI
        for i, (_,subDict) in enumerate(self.textBoxDict["Joint Space Control Settings"].items()):
            state = round(np.rad2deg(jState[i]),2)
            subDict["Current Position"].setText(str(state))
        
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
        