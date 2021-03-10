import constants as c
import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim


class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        if self.jointName == "Torso_Leg1":
            print("HALF OSCILLATION")
            self.frequency = c.frequency/2
        else:
            self.frequency = c.frequency
        self.offset = c.phaseOffset
        i = np.linspace(-np.pi, np.pi, 1000)
        self.motorValues = np.sin(self.frequency * i + self.offset) * self.amplitude

    def Set_Value(self, robot, timestamp):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex=robot.robot,
            jointName=self.jointName,
            controlMode=p.POSITION_CONTROL,
            targetPosition=self.motorValues[timestamp],
            maxForce=25
        )

    def Save_Value(self):
        np.save(f'data/{self.jointName}.npy', self.motorValues)
