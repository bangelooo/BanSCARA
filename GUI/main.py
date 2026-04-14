# Import Python Libraries
import sys
from PyQt6.QtWidgets import (
    QApplication
)

import numpy as np 

# Import Custom Libraries
from CustomLibraries.RobotArmLibrary import (
    createBanSCARA,
    RobotArmController
)
from CustomLibraries.RobotCommunications import (
    Topic,
    Action,
    ActionClient
)
from CustomLibraries.RobotGUI import RobotGUI

if __name__ =='__main__':
    # Instantiate Robot
    robotArm = createBanSCARA()

    # Create shared topics
    jStateTopic = Topic("jState")
    cStateTopic = Topic("cState")
    calStateTopic = Topic("calState")
    gripDiameterTopic = Topic("gripDiameter")

    topicsList = [jStateTopic,cStateTopic,calStateTopic,gripDiameterTopic]

    # Create shared Actions
    jCmdAction = Action("jCmd")
    cCmdAction = Action("cCmd")
    calCmdAction = Action("calCmd")
    gripObjAction = Action("gripCmd")
    releaseObjAction = Action("releaseCmd")


    actionsList = [jCmdAction,cCmdAction,calCmdAction,gripObjAction,releaseObjAction]
    serviceList = []

    # Instantiate Robot Controller
    robotControl = RobotArmController(robotArm,topicsList,serviceList,actionsList)
    #robotControl.runRoutine()

    app = QApplication(sys.argv)
    window = RobotGUI(robotArm,robotControl,topicsList,serviceList,actionsList)

    window.show()

    app.exec()
