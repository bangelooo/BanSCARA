from scaraRobot import scaraRobot

def bsDH(d1,theta2,theta3,theta4):
    return{
    "q1":[0,0,253.65 + d1, 0],
    "q2":[190,0,0,theta2],
    "q3":[150,0,0,theta3],
    "q4":[0,0,0,theta4],
    }

# Create initial DH table for SCARA robot
initDH = bsDH(0,0,0,0)

# Instantiation
robot = scaraRobot(initDH)

# Check forward kinematics and inverse kinematics functions work
fk = robot.forwardKinematics()
print(fk)
ik = robot.scaraIK(340,0,253.65,0,False)
print(ik)

ik = robot.scaraIK(150,190,253.65,0,True)
print(ik)