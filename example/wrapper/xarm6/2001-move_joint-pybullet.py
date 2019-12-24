#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
"""

import os
import sys
import time
import math

import pybullet as p
import pybullet_data as pd
p.connect(p.GUI)#, options="--background_color_red=1.0 --background_color_blue=1.0 --background_color_green=1.0")
p.setAdditionalSearchPath(pd.getDataPath())

useFixedBase = True
flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION

#plane_pos = [0,0,0]
#plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
table_pos = [0,0,-0.625]
table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
xarm = p.loadURDF("xarm/xarm6_robot.urdf", flags = flags, useFixedBase=useFixedBase)

jointIds = []
paramIds = []

for j in range(p.getNumJoints(xarm)):
  p.changeDynamics(xarm, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(xarm, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
  
skip_cam_frames = 10  



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


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.reset(wait=True)
arm.set_mode(1)
arm.set_state(state=0)
speed = math.radians(5)


while (p.isConnected()):
  p.stepSimulation()
  targetPositions=[]
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(xarm, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
    targetPositions.append(targetPos)
  skip_cam_frames -= 1
  arm.set_servo_angle_j(angles=targetPositions, speed=speed, is_radian=True)
  if (skip_cam_frames<0):
    p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL )
    skip_cam_frames = 20
  time.sleep(1./240.)


arm.reset(wait=True)
arm.disconnect()
