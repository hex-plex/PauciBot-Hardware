import pybullet as p
import pybullet_data
from time import sleep, time
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
kp, kd, ki = 0.005, 0.005, 0.000
init = time()
target_pos = 0
prev_error = None
encoder_pos = [0,0]
while(1):
	orie = p.getBasePositionAndOrientation(bot)[1]
	euler = p.getEulerFromQuaternion(orie)
	pitch = euler[1]
	dt = time()-init
	error = (pitch-target_pos)
	if prev_error is None:
		prev_error = error
	feedback = kp*error + kd*(error - prev_error)/dt + (ki*dt*error/abs(error+1e-6) if abs(error)<0.01 else 0)
	#print(feedback, pitch)
	feedback/=500
	encoder_pos[0]-=feedback
	encoder_pos[1]-=feedback
	p.setJointMotorControl2(bot, wheels[0], p.POSITION_CONTROL, targetPosition=encoder_pos[0], force=maxForce)# targetVelocity=-max(min(15,feedback),-15),)
	p.setJointMotorControl2(bot, wheels[1], p.POSITION_CONTROL, targetPosition=encoder_pos[1], force=maxForce)# targetVelocity=-max(min(15,feedback),-15), )
	#print(list(p.getJointState(bot, wheel) for wheel in wheels))
	p.stepSimulation()
	sleep(0.05)
	init = time()
