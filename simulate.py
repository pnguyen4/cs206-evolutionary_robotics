import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf") # the floor
p.loadSDF("box.sdf")

for i in range(0, 1000):
    p.stepSimulation()
    time.sleep(1/60)

p.disconnect()
