import pybullet as p
import pybullet_data

import numpy as np

p.connect(p.GUI) # alternatively p.connect(p.DIRECT) if you don't want GUI
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeUid=p.loadURDF("plane.urdf", [0, 0, 0])

# %% create the robot
id_ball1 = p.loadURDF('data/ball.urdf',
                      basePosition=[0,0,0],
                      baseOrientation=[0,0,0,1])

id_ball2 = p.loadURDF('data/ball.urdf',
                      basePosition=[0,0,0],
                      baseOrientation=[0,0,0,1])

while True:
    contactPoints = p.getContactPoints(id_ball1, id_ball2)
    if len(contactPoints) > 0:
        print(len(contactPoints))

    p.stepSimulation()
