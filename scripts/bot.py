import pybullet as p
import pybullet_data
from time import time
class Bot:
    wheels = [ 2, 5 ]
    maxVel = 15
    maxForce = 6
    target_pos = ((None, None, None),(None, 0, None)) ## ((target_x, target_y. target_z), (target_roll, target_yaw, target_pitch))
    ## None means the following variable would not be taken into consideration in control
    def __init__(self,url="urdf/Paucibot.urdf",botpos=[0.0, 0.0, 0.1], pClient=None):
        if pClient is None:
            pClient = 0
        self.pClient = pClient
        self.id = p.loadURDF(url,*botpos)
        self.control_instance = PiD_control()
    def setpoint(self,
                 target_pos = None,
                 target_x = None,
                 target_y = None,
                 target_z = None,
                 target_roll = None,
                 target_pitch = None,
                 target_yaw = None
                 ):
        '''
        This sets expected setpoint for the bot to traverse, if a particular state variable is None then control algo will ignore that as constraint.
        '''
        if target_pos is None:
            self.target_pos = ((target_x, target_y, target_z), (target_roll, target_pitch, target_yaw))
        else:
            self.target_pos = target_pos
    def control(self):
        '''
        This is responsible for the bots control algorithm using PID 
        '''
        pos, orie = p.getBasePositionAndOrientation(self.id, physicsClientId = self.pClient)
    	euler = p.getEulerFromQuaternion(orie)
        feedback = control_instance(pos, euler)
        p.setJointMotorControl2(self.id,
                                self.wheels[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity = -max(min(15,feedback),-15),
                                force = self.maxForce,
                                physicsClientId = self.pClient)
    	p.setJointMotorControl2(self.id,
                                self.wheels[1],
                                p.VELOCITY_CONTROL,
                                targetVelocity= -max(min(15,feedback),-15),
                                force = self.maxForce,
                                physicsClientId = self.pClient)
    def get_info(self):
        numJoints = p.getNumJoints(self.id, physicsClientId=self.pClient)
        return list(p.getJointInfo(self.id,joint, physicsClientId=self.pClient) for joint in range(numJoints))

class PiD_control:
    kp, kd, ki = 0.005, 0.005, 0.000
    init = None
    def __init__(self):
        pass
    def __call__(self, pos, euler):
        pass
    def set_param(self, Kp = None, Kd = None, Ki = None):
        self.kp, self.kd, self.ki = Kp or self.kp, Kd or self.kd, Ki or self.ki
