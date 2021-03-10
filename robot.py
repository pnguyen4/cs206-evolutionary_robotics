import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR


class ROBOT:
    def __init__(self):
        self.robot = p.loadURDF("body.urdf")    # the robot
        pyrosim.Prepare_To_Simulate("body.urdf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):
        self.sensors: dict[str, SENSOR] = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, timestep):
        for (_, s) in self.sensors.items():
            s.Get_Value(timestep)

    def Prepare_To_Act(self):
        self.motors: dict[str, MOTOR] = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self, timestamp):
        for (_, m) in self.motors.items():
            m.Set_Value(self, timestamp)
