import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
import os
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c


class ROBOT:
    def __init__(self, solutionID):
        self.solutionID = solutionID
        self.robot = p.loadURDF("body.urdf")    # the robot
        pyrosim.Prepare_To_Simulate("body.urdf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK(f"brain{solutionID}.nndf")
        os.system(f"rm brain{solutionID}.nndf")

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
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = \
                    self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(self, desiredAngle)
                # print(f'{neuronName}, {jointName}, {desiredAngle}')

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robot, 0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        # basePosAndOrient = p.getBasePositionAndOrientation(self.robot)
        # basePosition = basePosAndOrient[0]
        # xPosition = basePosition[0]
        f = open(f"tmp{self.solutionID}.txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.system(f"mv tmp{self.solutionID}.txt fitness{self.solutionID}.txt")
