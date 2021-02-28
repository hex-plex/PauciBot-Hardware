import pybullet as p
import pybullet_data
from time import sleep, time
from bot import Bot
pClient = p.connect(p.GUI)
print(pClient)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
botpos=[0,0,0.1]
bot = Bot()
#bot = p.loadURDF("urdf/Paucibot.urdf",*botpos)
p.setGravity(0,0,-10)
print(bot.get_info())
kp, kd, ki = 0.005, 0.005, 0.000
init = time()
target_pos = 0
prev_error = None
while(1):
	orie = p.getBasePositionAndOrientation(bot)[1]
	euler = p.getEulerFromQuaternion(orie)
	pitch = euler[1]
	dt = time()-init
	error = (pitch-target_pos)
	if prev_error is None:
		prev_error = error
	feedback = kp*error + kd*(error - prev_error)/dt + (ki*dt*error/abs(error+1e-6) if abs(error)<0.01 else 0)
	print(feedback, pitch)
	p.setJointMotorControl2(bot, wheels[0], p.VELOCITY_CONTROL, targetVelocity=-max(min(15,feedback),-15), force=maxForce)
	p.setJointMotorControl2(bot, wheels[1], p.VELOCITY_CONTROL, targetVelocity=-max(min(15,feedback),-15), force=maxForce)
	# Convert the code to POSITION_CONTROL as i am using PID on the motor with encoder
	p.stepSimulation()
	sleep(0.05)
	init = time()
