import pybullet as p
import pybullet_data
from time import sleep, time
from bot import Bot
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
botpos=[0,0,0.1]
bot = p.loadURDF("urdf/Paucibot.urdf",*botpos)
p.setGravity(0,0,-10)
numJoints = p.getNumJoints(bot)
for joint in range(numJoints):
	print(p.getJointInfo(bot,joint))
#car = Bot()
wheels = [ 2, 5 ]
targetVel = 15
maxForce = 6
kp, kd, ki = 0.05, 0.005, 0.000
init = time()
#target_pos = car.setpoint(target_roll = 0, target_yaw = 0, target_pitch =0)
target_pitch = 0
target_yaw = 0
target_roll = 0
prev_error_pitch = None
prev_error_yaw = None
prev_error_roll = None 
encoder_pos = [0,0]
encoder_vel = [0,0]
i=0
'''Bhayiya isme agar error in pitch 
kam hai toh yaw 0 ho raha hai now i am 
only working on pitch so that uska 
error kam ho sake'''
while(1):
	orie = p.getBasePositionAndOrientation(bot)[1]
	euler = p.getEulerFromQuaternion(orie)
	pitch = euler[1]
	yaw = euler[2]
	roll = euler[0]
	dt = time()-init
	error = (pitch-target_pitch)
	if prev_error_pitch is None:
		prev_error_pitch = error
	feedback1 = kp*error + kd*(error - prev_error_pitch)/dt + (ki*dt*error/abs(error+1e-6) if abs(error)<0.01 else 0)
	feedback1/=500
	error1 = yaw-target_yaw
	if prev_error_yaw is None:
		prev_error_yaw = error1
	feedback2 = kp*error1 + kd*(error1 - prev_error_yaw)/dt
	feedback2/=100 
	if(error<0.1 and error>-0.1):
		encoder_vel[0] = -feedback2
		encoder_vel[1] = feedback2
		print("yaw: ",error1, pitch)
		p.setJointMotorControl2(bot, wheels[0], p.VELOCITY_CONTROL, targetVelocity=encoder_vel[0], force=maxForce)# targetVelocity=-max(min(15,feedback),-15),)
		p.setJointMotorControl2(bot, wheels[1], p.VELOCITY_CONTROL, targetVelocity=encoder_vel[1], force=maxForce)# targetVelocity=-max(min(15,feedback),-15), 
		p.stepSimulation()
	else:
		encoder_pos[0] -= feedback1 
		encoder_pos[1] -= feedback1
		print("pitch: ", error, yaw)
		p.setJointMotorControl2(bot, wheels[0], p.POSITION_CONTROL, targetPosition=encoder_pos[0], force=maxForce)# targetVelocity=-max(min(15,feedback),-15),)
		p.setJointMotorControl2(bot, wheels[1], p.POSITION_CONTROL, targetPosition=encoder_pos[1], force=maxForce)# targetVelocity=-max(min(15,feedback),-15), )
		#print(list(p.getJointState(bot, wheel) for wheel in wheels))
		p.stepSimulation()
	i+=1
	sleep(0.05)
	init = time()
