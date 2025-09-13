import numpy as np

class scaraRobot:
	# Constructor
	def __init__(self,stdDHTable):
		# Attributes
		self.dhTable = stdDHTable
		self.numJoints = len(self.dhTable)
		
	# Methods
	def stdHT(self,a,alpha,d,theta):
		"""Create a homogeneous transformation ussing the """	
		# Convert degrees into radians
		theta = np.deg2rad(theta)
		alpha = np.deg2rad(alpha)
		
		# Create variables for cosine and sine
		ct,st = np.cos(theta) , np.sin(theta)
		ca,sa = np.cos(alpha) , np.sin(alpha)
		
		# Standard Denavit Hartenberg matrix
		HT = np.array([
			[ct, -st * ca,  st * sa, a * ct],
			[st,  ct * ca, -ct * sa, a * st],
			[0,        sa,       ca,      d],
			[0,         0,        0,      1]
			],dtype = float)
		return HT
	
	def forwardKinematics(self):
		"""Return the HT matrix from base to EE"""
		FK = np.eye(4) # 4X4 Indentity Matrix
		for joint in self.dhTable:
				Tc = self.stdHT(*self.dhTable[joint])
				FK = FK @ Tc
		return FK 
	
	def updateJoints(jointList):
		pass
	
	def scaraIK(self,x,y,z,phi,elbowUp = False):
		"""Inverse kinematics for for the banSCARA. Returns the joint angles (in degrees)
		to achieve the desired robot pose."""
		# Define Revolute Joints in SCARA
		a1 = self.dhTable["q2"][0]
		a2 = self.dhTable["q3"][0]
		a3 = self.dhTable["q4"][0]
		
		# Calculate P2X,P2Y
		p2x = x - a3 * np.cos(phi)
		p2y = y - a3 * np.sin(phi)
		
		# Calculate elbow
		q2Num = (p2x**2 + p2y**2) - (a1**2 + a2**2)
		q2Den = 2 * a1 * a2
		
		if elbowUp:
			theta2 = -np.arccos(q2Num/q2Den)
		else:
			theta2 = np.arccos(q2Num/q2Den)
		
		# Calculate shoulder
		q1Den = a1 + (a2 * np.cos(theta2))
		q1Num = a2 * np.sin(theta2)
		
		alpha = np.arctan2(p2y,p2x)
		beta = np.arctan2(q1Num,q1Den)
		
		theta1 = alpha - beta
		
		# Calculate wrist
		theta3 = np.deg2rad(phi) - theta2 - theta1
		
		# Convert joint angles from radians to degrees
		theta1 = np.rad2deg(theta1)
		theta2 = np.rad2deg(theta2)
		theta3 = np.rad2deg(theta3)
		
		# Calculate Vertical Height
		d1 = z - 253.65
		
		return d1,theta1,theta2,theta3
		