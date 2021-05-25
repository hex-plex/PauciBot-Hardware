import pybullet as p
import pybullet_data
from time import sleep, time
# import control 
# import slycot
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
botpos=[0,0,0.08]
botori = p.getQuaternionFromEuler([0, 0, 0])

bot = p.loadURDF("urdf/Paucibot.urdf", *botpos, *botori)
# bot = p.loadURDF("urdf/Paucibot.urdf",*botpos)
p.setGravity(0,0,-10)
numJoints = p.getNumJoints(bot)
for joint in range(numJoints):
	print(p.getJointInfo(bot,joint))
wheels = [ 2, 5 ]
targetVel = 15
maxForce = 6
kp, kd, ki = 255,26,34
init = time()
target_pitch = 0.0
target_yaw = 0
prev_error = 0
prev_error1 = 0
inti_term = 0
inti_term1 = 0
encoder_pos = [0,0]
encoder_vel = [0,0]
while(1):
	orie = p.getBasePositionAndOrientation(bot)[1]
	euler = p.getEulerFromQuaternion(orie)
	pitch = euler[1]
	yaw = euler[2]
	dt = time()-init
	error = (pitch-target_pitch)
	error1 = yaw -target_yaw
    #k = control.lqr(A,B,Q,R)
	if abs(error)<0.01:
		inti_term += error*dt
		if abs(error1)<0.01:
			inti_term1 += error1*dt
		else:
			inti_term1 = 0
		feedback1 = 1*error1 + 1*(error1 - prev_error1)/dt + 1*inti_term1
		prev_error1 = error1
		print("Yaw control - error:", error1," feedback:", feedback1)
		encoder_vel[0] = -feedback1
		encoder_vel[1] = feedback1
		p.setJointMotorControl2(bot, wheels[0], p.VELOCITY_CONTROL, targetVelocity=encoder_vel[0], force=maxForce)
		p.setJointMotorControl2(bot, wheels[1], p.VELOCITY_CONTROL, targetVelocity=encoder_vel[1], force=maxForce)
		p.stepSimulation()
	else:
		inti_term = 0
	feedback = 255*error + 26*(error - prev_error)/dt + 34*inti_term
	prev_error = error
	print("error: ", error)
	feedback/=500
    #feedback = - k * error
	print("feedback: ",feedback)
	encoder_pos[0] -= feedback
	encoder_pos[1] -= feedback
	p.setJointMotorControl2(bot, wheels[0], p.POSITION_CONTROL, targetPosition=encoder_pos[0], force=maxForce)# targetVelocity=-max(min(15,feedback),-15),)
	p.setJointMotorControl2(bot, wheels[1], p.POSITION_CONTROL, targetPosition=encoder_pos[1], force=maxForce)# targetVelocity=-max(min(15,feedback),-15), )
	#print(list(p.getJointState(bot, wheel) for wheel in wheels))
	p.stepSimulation()
	sleep(0.05)
	init = time()

