from symbolicRobot import symbolicRobot as sr
import sympy as sp

# Define robot and number of joints
robot = sr(numJoints = 4)

# Retrieve symoblic variables from robot
a,alpha,d,theta = robot.retrieveSymbols()

# Create DH Table
robot.addDHPar(0,0,253.65 + d[1], 0) # Joint 1
robot.addDHPar(a[2],0,0,theta[2]) # Joint 2
robot.addDHPar(a[3],0,0,theta[3]) # Joint 3
robot.addDHPar(0,0,0,theta[4]) # Joint 4

# Create Homogenous Transformation Matrices for Each Joint\
T01 = robot.stdHT(*robot.dhTable[0])
T12 = robot.stdHT(*robot.dhTable[1])
T23 = robot.stdHT(*robot.dhTable[2])
T34 = robot.stdHT(*robot.dhTable[3])

htList = [T01,T12,T23,T34]
for tMatrix in htList:
    robot.HT.append(tMatrix)

# Forward Kinematics
T04 = T01 * T12 * T23 * T34
T04_simplify = sp.simplify(T04)
#sp.pprint(T04_simplify)
sp.pprint(robot.extractRot(4))
# Jacobian