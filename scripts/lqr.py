import pybullet as p
import pybullet_data
from time import sleep, time 
from control import lqr
import numpy as np
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
botpos=[0,0,0.1]
bot = p.loadURDF("urdf/Paucibot.urdf",*botpos)
p.setGravity(0,0,-10)
numJoints = p.getNumJoints(bot)
for joint in range(numJoints):
	print(p.getJointInfo(bot,joint))
wheels = [ 2, 5 ]
targetVel = 15
maxForce = 6
A = [[38.7316,12.879,5.6979,9.9588],
	 [-104.7609,39.2818,18.0856,30.5764],
	 [83.2656,-30.5537,-13.2043,-23.5357],
	 [-44.2548,16.7696,6.3255,12.654]]

B = [[-0.7188],
	 [-2.0030],
	 [1.5985],
	 [-0.8379]]

Q = [[100,0,0,0],
     [0,1,0,0],
     [0,0,100,0],
     [0,0,0,1]]
R = 0.5
kp, kd, ki = 0.005, 0.005, 0.000
init = time()
target_pos = 0
prev_error = 0
inti_term = 0
encoder_pos = [0,0]
while(1):
	orie = p.getBasePositionAndOrientation(bot)[1]
	euler = p.getEulerFromQuaternion(orie)
	pitch = euler[1]
	dt = time()-init
	error = (pitch-target_pos)
	# if prev_error is None:
	# 	prev_error = error
	# feedback = kp*error + kd*(error - prev_error)/dt + (ki*dt*error/abs(error+1e-6) if abs(error)<0.01 else 0)
	k = lqr(A,B,Q,R)[0]
	print("k:   ")
	k = np.resize(k,(4,1))
	print(k)
	if abs(error)<0.01:
		inti_term += error*dt
	else:
		inti_term = 0
	feedback = -k[0] * error - k[1]*(error - prev_error) - k[2]*inti_term
	feedback/=500
	print("feedback: ", feedback)
	print("error: ", error)
	prev_error = error
	encoder_pos[0]-=feedback
	encoder_pos[1]-=feedback
	p.setJointMotorControl2(bot, wheels[0], p.POSITION_CONTROL, targetPosition=encoder_pos[0], force=maxForce)# targetVelocity=-max(min(15,feedback),-15),)
	p.setJointMotorControl2(bot, wheels[1], p.POSITION_CONTROL, targetPosition=encoder_pos[1], force=maxForce)# targetVelocity=-max(min(15,feedback),-15), )
	#print(list(p.getJointState(bot, wheel) for wheel in wheels))
	p.stepSimulation()
	sleep(0.05)
	init = time()