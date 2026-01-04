import sys
import numpy as np
from scaraFactory import createBanSCARA

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget,
    QPushButton, QLineEdit,QTextEdit, QLabel,
    QVBoxLayout,QHBoxLayout,QGridLayout,
    QGroupBox
)

class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Create Robot Instance
        self.robot = createBanSCARA()

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
            buttonGroupDict["button"].clicked.connect(self.sendJogCommand)

        for _, buttonGroupDict in self.buttonDict["Cartesian Control"].items():
            buttonGroupDict["button"].clicked.connect(self.sendCartCommand)

        self.buttonDict["String Command"]["output"]["button"].clicked.connect(self.clearLog)

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
        cCtrlGroup, cCtrlDict = self.createCCtrlGroup()
        jJogGroup,jJogDict = self.createJCtrlGroup()


        # Add buttons to desired dictionary
        self.buttonDict["Cartesian Control"] = cCtrlDict
        self.buttonDict["Joint Space Control"] = jJogDict

        # Add widget to control panel layout
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


    def sendJogCommand(self):
        sender = self.sender()

        # Update command ID
        self.cmdID += 1

        # Initilize jog parameters
        joint = ""
        jogDistance = 0
        direction = 1
        jointPositions = []

        # Search dictionary where button object is located and set jog parameters
        for _, buttonGroupDict in self.buttonDict["Joint Space Control"].items():
            if buttonGroupDict["button"] == sender:
                joint = buttonGroupDict["joint"]
                if "direction" in buttonGroupDict:
                    direction= buttonGroupDict["direction"]
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
        
        # Send Command
        if joint == "all":
            strCMD =f"<ABS, Q1:{jointPositions[0]},Q2:{jointPositions[1]},Q3:{jointPositions[2]},Q4:{jointPositions[3]}>"
            self.updateLog(strCMD)

        else:
            strCMD = f"<REL,{joint}: {jogDistance * direction}>"
            self.updateLog(strCMD)
        
    def sendCartCommand(self):
        self.updateRobotState()

        self.cmdID += 1

        # Identify pushed button
        sender = self.sender()

        # Intialize required variables
        axis = ""
        jogDistance = 0
        direction = 1
        poseStr = []

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
        
        # 1. Get Current Pose
        # 2. Add increment the desired DOF
        # 3. Inverse Kinematics
        # 4. Command robot

        # Get lastet desired pose
        for DOF in ["X","Y","Z","φ"]:
            desPose = self.textBoxDict["Cartesian Control Settings"][DOF]["Desired Pose"].text()
            poseStr.append(desPose)
        
        pose = [float(item) for item in poseStr]
        
        if axis == "all":
            Q1,Q2,Q3,Q4 = self.robot.scaraIK(pose[0],pose[1],pose[2],pose[3],'elbowUp')            
            print(f"<{self.cmdID},ABS,q1:{Q1},q2:{Q2},q3:{Q3},q4:{Q4}>")
        else:
            print(f"{self.cmdID},REL,q{axis}:{direction * jogDistance}")

# ==================================================================  
#     METHODS TO TEXT FIELD BEHAVIORS
# ==================================================================  
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

app = QApplication(sys.argv)

window = RobotGUI()
window.show()

app.exec()