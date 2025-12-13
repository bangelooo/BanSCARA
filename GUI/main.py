import numpy as np
from robotArm import robotArm
from robotArm import robotLink

# Declare robot link parametrs 
L1,L2,L3,L4 = 1,1,1,0.5
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

# Define Trajectories
phi = [np.pi/2,np.pi/4]
phiTraj = np.linspace(phi[0],phi[1],100)

X = [[1.5,-1],[1.5,1.5],[0,0]]

# Simulate Robot
scaraRobot.animateTraj(X,phi,"elbowUp",100)
