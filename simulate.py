import numpy as np
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")  # the floor
bodyId = p.loadURDF("body.urdf")    # the robot
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate("body.urdf")
backLegSensorValues = np.zeros(10000)
for i in range(0, 1000):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    time.sleep(1/60)

p.disconnect()
print(backLegSensorValues)
