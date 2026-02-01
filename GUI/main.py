# Import Python Libraries
import sys
from PyQt6.QtWidgets import (
    QApplication
)
# Import Custom Libraries
from CustomLibraries.RobotArmLibrary import (
    createBanSCARA,
    RobotArmController
)
from CustomLibraries.RobotCommunications import (
    Topic
)
from CustomLibraries.RobotGUI import RobotGUI



# Instantiate Robot
robotArm = createBanSCARA()

# Create shared topics
jStateTopic = Topic("joint_state")
cStateTopic = Topic("cartesian_state")
jCmdTopic = Topic("joint_command")
cCmdTopic = Topic("cart_command")

# Instantiate Robot Controller
robotControl = RobotArmController(robotArm,jState = jStateTopic,cState = cStateTopic,jCmd = jCmdTopic, cCmd = cCmdTopic)

app = QApplication(sys.argv)
window = RobotGUI(robotArm,robotControl,jState = jStateTopic,cState = cStateTopic,jCmd = jCmdTopic, cCmd = cCmdTopic)
window.show()

app.exec()

