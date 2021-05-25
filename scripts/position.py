import pybullet as p
import pybullet_data
from time import sleep, time
import numpy as np

def magnitude(a):
    return (a[0]**2+a[1]**2)**0.5

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
target_x = 0
target_pitch =0
prev_error = 0
prev_error_pitch = 0
encoder_pos = [0,0]
encoder_vel = [0,0]
while(1):
    posi = p.getBasePositionAndOrientation(bot)[0]
    orie = p.getBasePositionAndOrientation(bot)[1]
    euler = p.getEulerFromQuaternion(orie)
    pitch = euler[1]
    dt = time()-init
    p1 = p.getLinkState(bot,9)[0]
    p2 = p.getLinkState(bot,10)[0]
    p3 = p.getLinkState(bot,11)[0]
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    direc = p1- (p2+p3)/2
    sum = direc + posi
    sum = magnitude(sum)
    direc = magnitude(direc)
    posi = magnitude(posi)
    error = sum - direc -posi
    feedback = kp*error + kd*(error - prev_error)/dt
    feedback/=10
    error_pitch = (pitch-target_pitch)
    feedback1 = kp*error_pitch + kd*(error_pitch - prev_error_pitch)/dt + (ki*dt*error/abs(error+1e-6) if abs(error)<0.01 else 0)
    prev_error_pitch = error_pitch
    feedback1/=500
    if (error_pitch<0.01 and error_pitch>-0.01):
        encoder_vel[0] = -feedback
        encoder_vel[1] = feedback
        print("direction: ", feedback, error)
        p.setJointMotorControl2(bot, wheels[0], p.VELOCITY_CONTROL, targetVelocity=encoder_vel[0], force=maxForce)# targetVelocity=-max(min(15,feedback),-15),)
        p.setJointMotorControl2(bot, wheels[1], p.VELOCITY_CONTROL, targetVelocity=encoder_vel[1], force=maxForce)# targetVelocity=-max(min(15,feedback),-15), )
    else:
        encoder_pos[0]-=feedback1 
        encoder_pos[1]-=feedback1 
        print("pitch: ", feedback1, error_pitch)
        p.setJointMotorControl2(bot, wheels[0], p.POSITION_CONTROL, targetPosition=encoder_pos[0], force=maxForce)# targetVelocity=-max(min(15,feedback),-15),)
        p.setJointMotorControl2(bot, wheels[1], p.POSITION_CONTROL, targetPosition=encoder_pos[1], force=maxForce)# targetVelocity=-max(min(15,feedback),-15), )
	#print(list(p.getJointState(bot, wheel) for wheel in wheels))
    prev_error = error
    p.stepSimulation()
    sleep(0.05)
    init = time()

