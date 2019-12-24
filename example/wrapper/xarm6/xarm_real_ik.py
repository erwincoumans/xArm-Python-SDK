import os
import sys
import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import xarm_sim
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI
#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

arm.set_mode(1)
arm.set_state(state=0)

speed = math.radians(150)


timeStep=1./60.
p.setTimeStep(timeStep)
p.setGravity(0,0,-9.8)
 
xarm = xarm_sim.XArm6Sim(p,[0,0,0])
while (p.isConnected()):
	xarm.step()
	arm.set_servo_angle_j(angles=xarm.jointPoses, speed=speed, is_radian=True)
	p.stepSimulation()
	time.sleep(timeStep)
	
arm.reset(wait=True)
arm.disconnect()
