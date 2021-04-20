import numpy as np
import os
import pyrosim.pyrosim as pyrosim
import random
import time
import constants as c


class SOLUTION:
    def __init__(self, id):
        self.myID = id
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons) * 2 - 1

    # def Evaluate(self, arg):
    #     pass

    def Start_Simulation(self, arg):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        # pybullet warnings and initialization output is supressed
        os.system(f"python3 simulate.py {arg} {self.myID} > /dev/null 2>&1 &")
        # os.system(f"python3 simulate.py {arg} {self.myID} &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists(f"fitness{self.myID}.txt"):
            time.sleep(0.01)
        f = open(f"fitness{self.myID}.txt", "r")
        self.fitness = float(f.read())
        f.close()
        os.system(f"rm fitness{self.myID}.txt")

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[5, 5, 0.5], size=[1, 1, 1])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        # How cube size dimensions work
        # ------------------------------------------------------
        # first number  (x) : red axis,   leftback to rightfront
        # second number (y):  green axis, rightback to leftfront
        # third number  (z):  blue axis,  up-down

        # How parent cube position coordinates work
        # ------------------------------------------------
        # They denote the location of the *center* of the
        # cube, according to *absolute* position 3D space.

        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 5], size=[1, 0.5, 2])

        # How joint position coordinates work
        # ----------------------------------------------
        # They denote where point of rotation, according
        # to *absolute* position in 3D space.

        # How child cube position coordinates work
        # -----------------------------------------------
        # They denote the location of the *center* of the
        # cube, according to *relative* position to joint.

        pyrosim.Send_Joint("LeftHip", "Torso", "TopLeftLeg",
                           "revolute", position="0.5 0 4.2", jointAxis="1 0 0")
        pyrosim.Send_Cube("TopLeftLeg", pos=[0.1, 0, -1], size=[0.2, 0.2, 2])

        pyrosim.Send_Joint("RightHip", "Torso", "TopRightLeg",
                           "revolute", position="-0.5 0 4.2", jointAxis="1 0 0")
        pyrosim.Send_Cube("TopRightLeg", pos=[-.1, 0, -1], size=[0.2, 0.2, 2])

        pyrosim.Send_Joint("LeftKnee", "TopLeftLeg", "BtmLeftLeg",
                           "revolute", position="0 0 -1.8", jointAxis="1 0 0")
        pyrosim.Send_Cube("BtmLeftLeg", pos=[0.1, 0, -1], size=[0.2, 0.2, 2])

        pyrosim.Send_Joint("RightKnee", "TopRightLeg", "BtmRightLeg",
                           "revolute", position="0 0 -1.8", jointAxis="1 0 0")
        pyrosim.Send_Cube("BtmRightLeg", pos=[-.1, 0, -1], size=[0.2, 0.2, 2])

        pyrosim.Send_Joint("LeftShoulder", "Torso", "TopLeftArm",
                           "revolute", position="0.5 0 5.8", jointAxis="1 0 0")
        pyrosim.Send_Cube("TopLeftArm", pos=[0.1, 0.5, 0], size=[0.2, 1, 0.2])

        pyrosim.Send_Joint("RightShoulder", "Torso", "TopRightArm",
                           "revolute", position="-.5 0 5.8", jointAxis="1 0 0")
        pyrosim.Send_Cube("TopRightArm", pos=[-.1, 0.5, 0], size=[0.2, 1, 0.2])

        pyrosim.Send_Joint("LeftElbow", "TopLeftArm", "BtmLeftArm",
                           "revolute", position="0 0.9 0", jointAxis="1 0 0")
        pyrosim.Send_Cube("BtmLeftArm", pos=[0.1, 0.5, 0], size=[0.2, 1, 0.2])

        pyrosim.Send_Joint("RightElbow", "TopRightArm", "BtmRightArm",
                           "revolute", position="0 0.9 0", jointAxis="1 0 0")
        pyrosim.Send_Cube("BtmRightArm", pos=[-.1, 0.5, 0], size=[0.2, 1, 0.2])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        # pyrosim.Send_Sensor_Neuron(name=1, linkName="LeftLeg")

        pyrosim.Send_Motor_Neuron(name=1, jointName="LeftHip")

        # for currentRow in range(0, c.numSensorNeurons):
        #     for currentColumn in range(0, c.numMotorNeurons):
        #         # wt = self.weights[currentRow][currentColumn]
        #         wt = 0
        #         pyrosim.Send_Synapse(currentRow,
        #                              currentColumn + c.numSensorNeurons,
        #                              weight=wt)

        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons-1)
        randomColumn = random.randint(0, c.numMotorNeurons-1)
        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, id):
        self.myID = id
