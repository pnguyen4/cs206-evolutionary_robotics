import numpy as np
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")  # the floor
robot = p.loadURDF("body.urdf")    # the robot
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate("body.urdf")
backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)

amplitude_bleg = np.pi/4
frequency_bleg = 20
phaseOffset_bleg = 0
i_bleg = np.linspace(-np.pi, np.pi, 1000)
targetAngles_bleg = np.sin(frequency_bleg * i_bleg + phaseOffset_bleg) * amplitude_bleg
# np.save('data/targetAngles_bleg.npy', targetAngles_bleg)

amplitude_fleg = np.pi/4
frequency_fleg = 10
phaseOffset_fleg = 0
i_fleg = np.linspace(-np.pi, np.pi, 1000)
targetAngles_fleg = np.sin(frequency_fleg * i_fleg + phaseOffset_fleg) * amplitude_fleg
# np.save('data/targetAngles_fleg.npy', targetAngles_fleg)

for i in range(0, 1000):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robot,
        jointName="Torso_Leg1",  # front leg
        controlMode=p.POSITION_CONTROL,
        targetPosition=targetAngles_fleg[i],
        maxForce=25
    )
    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robot,
        jointName="Torso_Leg2",  # back leg
        controlMode=p.POSITION_CONTROL,
        targetPosition=targetAngles_bleg[i],
        maxForce=25
    )
    time.sleep(1/60)

np.save('data/back_leg_sensor_values.npy', backLegSensorValues)
np.save('data/front_leg_sensor_values.npy', frontLegSensorValues)
p.disconnect()
# print(backLegSensorValues)
# print(frontLegSensorValues)
