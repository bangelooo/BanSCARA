# Import Python Libraries
import sys
from PyQt6.QtWidgets import (
    QApplication
)
# Import Custom Libraries
from CustomLibraries.robotGUI import RobotGUI
from CustomLibraries.scaraFactory import createBanSCARA
from CustomLibraries.RobotController import RobotController
from CustomLibraries.PubSub import Topic,Publisher, Subscriber

# Instantiate Robot
robotArm = createBanSCARA()

# Create shared topics
jStateTopic = Topic("joint_state")
cStateTopic = Topic("cartesian_state")
jCmdTopic = Topic("joint_command")
cCmdTopic = Topic("cart_command")

# Instantiate Robot Controller
robotControl = RobotController(robotArm,jState = jStateTopic,cState = cStateTopic)

app = QApplication(sys.argv)
window = RobotGUI(robotArm,robotControl)
window.show()

app.exec()

