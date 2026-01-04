import numpy as np
from CustomLibraries.robotArm import robotArm,robotLink

def createBanSCARA():
    # Declare robot link parametrs 
    L1,L2,L3,L4 = 1,191,191,0
    d4 = 1

    # Declare Links using Standard DH paramters
    link1 = robotLink(0,0,L1,0,"prismatic","std")
    link2 = robotLink(0,L2,0,0,"revolute","std")
    link3 = robotLink(0,L3,0,0,"revolute","std")
    linkEE = robotLink(-np.pi,L4,d4,0,"revolute","std")


    # Group robot links
    robotlinks = [link1,link2,link3,linkEE]

    # Create robot 
    scaraRobot = robotArm(robotlinks)

    return scaraRobot
