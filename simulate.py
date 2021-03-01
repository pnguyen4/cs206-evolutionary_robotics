import numpy as np
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import math

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")  # the floor
robot = p.loadURDF("body.urdf")    # the robot
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate("body.urdf")
backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)
for i in range(0, 1000):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robot,
        jointName="Torso_Leg1",  # front leg
        controlMode=p.POSITION_CONTROL,
        targetPosition=math.pi/6.0,
        maxForce=500
    )
    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robot,
        jointName="Torso_Leg2",  # back leg
        controlMode=p.POSITION_CONTROL,
        targetPosition=-math.pi/6.0,
        maxForce=500
    )
    time.sleep(1/60)

np.save('data/back_leg_sensor_values.npy', backLegSensorValues)
np.save('data/front_leg_sensor_values.npy', frontLegSensorValues)
p.disconnect()
# print(backLegSensorValues)
# print(frontLegSensorValues)
