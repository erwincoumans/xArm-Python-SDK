import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import xarm_sim

options = ""#"--mp4=\"video.mp4\""

p.connect(p.GUI, options=options)
p.setAdditionalSearchPath(pd.getDataPath())

timeStep=1./60.
p.setTimeStep(timeStep)
p.setGravity(0,0,-9.8)
 
xarm = xarm_sim.XArm7Sim(p,[0,0,0])
while (1):
	xarm.step()
	p.stepSimulation()
	time.sleep(timeStep)
	
